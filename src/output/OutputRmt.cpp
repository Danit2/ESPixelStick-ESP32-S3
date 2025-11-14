/*
* OutputRmt.cpp - driver code for ESPixelStick RMT Channel
*
* Project: ESPixelStick - An ESP8266 / ESP32 and E1.31 based pixel driver
* Copyright (c) 2015, 2024 Shelby Merrick
* http://www.forkineye.com
*
* Adapted to use IDF4-style RMT function API (driver/rmt.h) so it compiles with
* platform-espressif32 @ 6.12.0 and runs on ESP32-S3 without direct register access.
*/
#include "ESPixelStick.h"
#ifdef ARDUINO_ARCH_ESP32
#include "output/OutputRmt.hpp"
#include <driver/rmt.h>
#include <vector>
#include <cstdlib>

// forward declaration for the isr handler (not used for register operations anymore)
static void IRAM_ATTR rmt_intr_handler(void* param) { (void)param; }

// keep an array of pointers similar to original design
static c_OutputRmt * rmt_isr_ThisPtrs[MAX_NUM_RMT_CHANNELS] = { nullptr };

#ifdef USE_RMT_DEBUG_COUNTERS
static uint32_t RawIsrCounter = 0;
#endif // def USE_RMT_DEBUG_COUNTERS

static TaskHandle_t SendFrameTaskHandle = NULL;
static BaseType_t xHigherPriorityTaskWoken = pdTRUE;
static uint32_t FrameCompletes = 0;
static uint32_t FrameTimeouts = 0;

// small struct passed to watcher task (not used in blocking mode but kept for compatibility)
struct TransmitWatcherParam {
    int channel;
    rmt_item32_t * items;
    size_t count;
};

// watcher task: waits until RMT tx done, notifies send task and frees the buffer
static void TransmitWatcherTask(void * arg)
{
    TransmitWatcherParam * p = (TransmitWatcherParam*)arg;
    int ch = p->channel;
    // wait until the RMT transmitter finishes for this channel
    rmt_wait_tx_done((rmt_channel_t)ch, portMAX_DELAY);
    // notify the sendframe task that the transmission finished (to mimic original behavior)
    if (SendFrameTaskHandle)
    {
        vTaskNotifyGiveFromISR(SendFrameTaskHandle, &xHigherPriorityTaskWoken);
    }
    // free the items memory
    free(p->items);
    free(p);
    vTaskDelete(NULL);
}

//----------------------------------------------------------------------------
// Simple send task that polls all channels and triggers StartNewFrame()
void RMT_Task(void *arg)
{
    (void)arg;
    while (1)
    {
        // Give the outputs a chance to catch up.
        delay(1);
        // process all possible channels
        for (c_OutputRmt * pRmt : rmt_isr_ThisPtrs)
        {
            // do we have a driver on this channel?
            if (nullptr != pRmt)
            {
                // invoke the channel - StartNextFrame returns true if a frame was started
                if (pRmt->StartNextFrame())
                {
                    // wait for notification that the frame finished (emulates old behavior)
                    uint32_t NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
                    if (1 == NotificationValue)
                    {
                        ++FrameCompletes;
                    }
                    else
                    {
                        ++FrameTimeouts;
                    }
                }
            }
        }
    }
} // RMT_Task

//----------------------------------------------------------------------------
// Constructor / Destructor
c_OutputRmt::c_OutputRmt()
{
    memset((void *)&Intensity2Rmt[0], 0x00, sizeof(Intensity2Rmt));
    memset((void *)&SendBuffer[0], 0x00, sizeof(SendBuffer));

#ifdef USE_RMT_DEBUG_COUNTERS
    memset((void *)&BitTypeCounters[0], 0x00, sizeof(BitTypeCounters));
#endif // def USE_RMT_DEBUG_COUNTERS
} // c_OutputRmt

c_OutputRmt::~c_OutputRmt()
{
    if (HasBeenInitialized)
    {
        String Reason = (F("Shutting down an RMT channel requires a reboot"));
        RequestReboot(Reason, 100000);

        int ch = (int)OutputRmtConfig.RmtChannelId;
        // disable channel and driver for that channel
        #if defined(rmt_set_gpio)
            rmt_set_gpio((rmt_channel_t)ch, RMT_MODE_TX, (gpio_num_t)OutputRmtConfig.DataPin, false);
        #endif
        rmt_driver_uninstall((rmt_channel_t)ch);

        rmt_isr_ThisPtrs[(int)OutputRmtConfig.RmtChannelId] = (c_OutputRmt*)nullptr;
    }
} // ~c_OutputRmt

//----------------------------------------------------------------------------
// Begin: initialize the RMT channel and driver
void c_OutputRmt::Begin(OutputRmtConfig_t config, c_OutputCommon * _pParent)
{
    do // once
    {
        if (HasBeenInitialized)
        {
            // release the old GPIO pin.
            ResetGpio(OutputRmtConfig.DataPin);
        }

        // save the new config
        OutputRmtConfig = config;

        #if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
        if ((nullptr == OutputRmtConfig.pPixelDataSource) && (nullptr == OutputRmtConfig.pSerialDataSource))
        #else
        if (nullptr == OutputRmtConfig.pPixelDataSource)
        #endif
        {
            String Reason = (F("Invalid RMT configuration parameters. Rebooting"));
            RequestReboot(Reason, 10000);
            break;
        }

        // number of rmt slots per intensity (intensity = one byte in Option B)
        NumRmtSlotsPerIntensityValue = OutputRmtConfig.IntensityDataWidth + ((OutputRmtConfig.SendInterIntensityBits) ? 1 : 0);

        if (OutputRmtConfig_t::DataDirection_t::MSB2LSB == OutputRmtConfig.DataDirection)
        {
            TxIntensityDataStartingMask = 1 << (OutputRmtConfig.IntensityDataWidth - 1);
        }
        else
        {
            TxIntensityDataStartingMask = 1;
        }

        // Configure RMT channel using IDF-style API
        rmt_config_t RmtConfig;
        memset(&RmtConfig, 0, sizeof(RmtConfig));
        RmtConfig.rmt_mode = rmt_mode_t::RMT_MODE_TX;
        RmtConfig.channel = OutputRmtConfig.RmtChannelId;
        RmtConfig.gpio_num = OutputRmtConfig.DataPin;
        RmtConfig.clk_div = (uint32_t)RMT_Clock_Divisor;
        RmtConfig.mem_block_num = rmt_reserve_memsize_t::RMT_MEM_64;
        RmtConfig.tx_config.carrier_freq_hz = 10; // avoid zero due to driver bug
        RmtConfig.tx_config.carrier_level = rmt_carrier_level_t::RMT_CARRIER_LEVEL_LOW;
        RmtConfig.tx_config.carrier_duty_percent = 50;
        RmtConfig.tx_config.idle_level = OutputRmtConfig.idle_level;
        RmtConfig.tx_config.carrier_en = false;
        RmtConfig.tx_config.loop_en = false;
        RmtConfig.tx_config.idle_output_en = true;

        ResetGpio(OutputRmtConfig.DataPin);
        ESP_ERROR_CHECK(rmt_config(&RmtConfig));
        ESP_ERROR_CHECK(rmt_driver_install(OutputRmtConfig.RmtChannelId, 0, 0));

        // --- Extended channel setup & validation ---
        {
            rmt_channel_t ch = OutputRmtConfig.RmtChannelId;

            logcon(String("[RMT] Init Channel ") + String((int)OutputRmtConfig.RmtChannelId) +
                " Pin=" + String((int)OutputRmtConfig.DataPin) +
                " Tick=" + String(RMT_TickLengthNS, 2) + "ns" +
                " IdleBits=" + String(OutputRmtConfig.NumIdleBits));

            UpdateBitXlatTable(OutputRmtConfig.CitrdsArray);
            bool ok = ValidateBitXlatTable(OutputRmtConfig.CitrdsArray);
            logcon(String("[RMT] ValidateBitXlatTable ch ") + String((int)ch) + (ok ? " OK" : " DONE (check logs)"));

            #if defined(rmt_set_gpio)
                rmt_set_gpio(ch, RMT_MODE_TX, (gpio_num_t)OutputRmtConfig.DataPin, true);
            #endif
        }

        logcon(String("[RMT] Init Channel ") + String((int)config.RmtChannelId) +
            " Pin=" + String((int)config.DataPin) +
            " Tick=" + String(RMT_TickLengthNS, 2) + "ns");

        #if defined(SOC_RMT_SUPPORT_REF_TICK)
            ESP_ERROR_CHECK(rmt_set_source_clk(OutputRmtConfig.RmtChannelId, RMT_BASECLK_APB));
        #endif

        // reset the internal indices & buffer counters
        ISR_ResetRmtBlockPointers();
        memset((void*)&SendBuffer[0], 0x0, sizeof(SendBuffer));

        UpdateBitXlatTable(OutputRmtConfig.CitrdsArray);

        if (!SendFrameTaskHandle)
        {
            xTaskCreatePinnedToCore(RMT_Task, "RMT_Task", 4096, NULL, 5, &SendFrameTaskHandle, 1);
            vTaskPrioritySet(SendFrameTaskHandle, 5);
        }
        pParent = _pParent;
        rmt_isr_ThisPtrs[(int)OutputRmtConfig.RmtChannelId] = this;

        HasBeenInitialized = true;
    } while (false);
} // Begin

//----------------------------------------------------------------------------
// UpdateBitXlatTable & Validate
void c_OutputRmt::UpdateBitXlatTable(const CitrdsArray_t * CitrdsArray)
{
    if (nullptr != OutputRmtConfig.CitrdsArray)
    {
        const ConvertIntensityToRmtDataStreamEntry_t *CurrentTranslation = CitrdsArray;
        while (CurrentTranslation->Id != RmtDataBitIdType_t::RMT_LIST_END)
        {
            SetIntensity2Rmt(CurrentTranslation->Translation, CurrentTranslation->Id);
            CurrentTranslation++;
        }
    }
    else
    {
        logcon(String(CN_stars) + F(" ERROR: Missing pointer to RMT bit translation values (1) ") + CN_stars);
    }
} // UpdateBitXlatTable

bool c_OutputRmt::ValidateBitXlatTable(const CitrdsArray_t * CitrdsArray)
{
    bool Response = true;
    if (nullptr != OutputRmtConfig.CitrdsArray)
    {
        const ConvertIntensityToRmtDataStreamEntry_t *CurrentTranslation = CitrdsArray;
        while (CurrentTranslation->Id != RmtDataBitIdType_t::RMT_LIST_END)
        {
            SetIntensity2Rmt(CurrentTranslation->Translation, CurrentTranslation->Id);

            if (Intensity2Rmt[CurrentTranslation->Id].val != CurrentTranslation->Translation.val)
            {
                logcon(String(CN_stars) + F("ERROR: incorrect bit translation detected. Chan: ") + String((int)OutputRmtConfig.RmtChannelId) +
                    F(" Slot: ") + String(int(CurrentTranslation->Id)) +
                    F(" Got: 0x") + String(Intensity2Rmt[CurrentTranslation->Id].val, HEX) +
                    F(" Expected: 0x") + String(CurrentTranslation->Translation.val));
                Response = false;
            }

            CurrentTranslation++;
        }
    }
    else
    {
        logcon(String(CN_stars) + F("ERROR: Missing pointer to RMT bit translation values (2)") + CN_stars);
        Response = false;
    }
    return Response;
} // ValidateBitXlatTable

//----------------------------------------------------------------------------
// Status
void c_OutputRmt::GetStatus(ArduinoJson::JsonObject& jsonStatus)
{
    jsonStatus[F("NumRmtSlotOverruns")] = NumRmtSlotOverruns;
#ifdef USE_RMT_DEBUG_COUNTERS
    jsonStatus[F("OutputIsPaused")] = OutputIsPaused;
    JsonObject debugStatus = jsonStatus["RMT Debug"].to<JsonObject>();
    debugStatus["RmtChannelId"] = (int)OutputRmtConfig.RmtChannelId;
    debugStatus["GPIO"] = (int)OutputRmtConfig.DataPin;
    debugStatus["ErrorIsr"] = ErrorIsr;
    debugStatus["FrameCompletes"] = String(FrameCompletes);
    debugStatus["FrameStartCounter"] = FrameStartCounter;
    debugStatus["FrameTimeouts"] = String(FrameTimeouts);
    debugStatus["FailedToSendAllData"] = String(FailedToSendAllData);
    debugStatus["IncompleteFrame"] = IncompleteFrame;
    debugStatus["IntensityValuesSent"] = IntensityValuesSent;
    debugStatus["IntensityValuesSentLastFrame"] = IntensityValuesSentLastFrame;
    debugStatus["IntensityBitsSent"] = IntensityBitsSent;
    debugStatus["IntensityBitsSentLastFrame"] = IntensityBitsSentLastFrame;
    debugStatus["IntTxEndIsrCounter"] = IntTxEndIsrCounter;
    debugStatus["IntTxThrIsrCounter"] = IntTxThrIsrCounter;
    debugStatus["ISRcounter"] = ISRcounter;
    debugStatus["NumIdleBits"] = OutputRmtConfig.NumIdleBits;
    debugStatus["NumFrameStartBits"] = OutputRmtConfig.NumFrameStartBits;
    debugStatus["NumFrameStopBits"] = OutputRmtConfig.NumFrameStopBits;
    debugStatus["NumRmtSlotsPerIntensityValue"] = NumRmtSlotsPerIntensityValue;
    debugStatus["OneBitValue"] = String(Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID].val, HEX);
    debugStatus["RanOutOfData"] = RanOutOfData;
    debugStatus["RawIsrCounter"] = RawIsrCounter;
    debugStatus["RmtEntriesTransfered"] = RmtEntriesTransfered;
    debugStatus["RmtWhiteDetected"] = String(RmtWhiteDetected);
    debugStatus["RmtXmtFills"] = RmtXmtFills;
    debugStatus["RxIsr"] = RxIsr;
    debugStatus["SendBlockIsrCounter"] = SendBlockIsrCounter;
    debugStatus["SendInterIntensityBits"] = OutputRmtConfig.SendInterIntensityBits;
    debugStatus["SendEndOfFrameBits"] = OutputRmtConfig.SendEndOfFrameBits;
    debugStatus["TX END int_ena"] = "N/A";
    debugStatus["TX END int_st"] = "N/A";
    debugStatus["TX THRSH int_ena"] = "N/A";
    debugStatus["TX THRSH int_st"] = "N/A";
    debugStatus["UnknownISRcounter"] = UnknownISRcounter;
    debugStatus["ZeroBitValue"] = String(Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID].val, HEX);
#endif // def USE_RMT_DEBUG_COUNTERS
} // GetStatus

//----------------------------------------------------------------------------
// ISR_CreateIntensityData - convert intensity bytes into RMT items (Option B)
void IRAM_ATTR c_OutputRmt::ISR_CreateIntensityData()
{
    // Solange noch Platz im Buffer (reserve für Terminator) und noch Daten
    while (NumUsedEntriesInSendBuffer < (NUM_RMT_SLOTS - 4))
    {
        uint32_t intensityByte = 0;

        // ISR_GetNextIntensityToSend liefert das Byte und als Rückgabewert, ob danach noch Daten sind.
        bool moreAfter = ISR_GetNextIntensityToSend(intensityByte);

        // Sende das gerade gelesene Byte IMMER (auch wenn moreAfter == false)
        uint32_t mask;
        if (OutputRmtConfig.DataDirection == OutputRmtConfig_t::DataDirection_t::MSB2LSB)
        {
            mask = 0x80u; // MSB first
        }
        else
        {
            mask = 0x01u; // LSB first
        }

        for (uint32_t b = 0; b < OutputRmtConfig.IntensityDataWidth; ++b)
        {
            bool isOne;
            if (OutputRmtConfig.DataDirection == OutputRmtConfig_t::DataDirection_t::MSB2LSB)
            {
                isOne = (intensityByte & mask) != 0;
                mask >>= 1;
            }
            else
            {
                isOne = (intensityByte & mask) != 0;
                mask <<= 1;
            }

            // write the appropriate translation slot
            ISR_WriteToBuffer(isOne
                ? Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID].val
                : Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID].val);
        }

        // Falls konfiguriert, sende ein InterIntensity-Bit (z. B. separater Stop/Start)
        if (OutputRmtConfig.SendInterIntensityBits)
        {
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_STOPBIT_ID].val);
        }

        // Setze ThereIsDataToSend nach dem Senden des Bytes
        ThereIsDataToSend = moreAfter;

        // Wenn keine weiteren Daten mehr anstehen, beende das Erzeugen
        if (!ThereIsDataToSend)
        {
            // Optional End-of-frame marker
            if (OutputRmtConfig.SendEndOfFrameBits)
            {
                ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_END_OF_FRAME].val);
            }
            break;
        }
    } // while buffer has room
} // ISR_CreateIntensityData

//----------------------------------------------------------------------------
// ISR_GetNextIntensityToSend - delegate to pixel or serial source
inline bool IRAM_ATTR c_OutputRmt::ISR_GetNextIntensityToSend(uint32_t &DataToSend)
{
    if (nullptr != OutputRmtConfig.pPixelDataSource)
    {
        return OutputRmtConfig.pPixelDataSource->ISR_GetNextIntensityToSend(DataToSend);
    }
#if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
    else
    {
        return OutputRmtConfig.pSerialDataSource->ISR_GetNextIntensityToSend(DataToSend);
    }
#else
    return false;
#endif // defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
} // ISR_GetNextIntensityToSend

//----------------------------------------------------------------------------
// ISR_Handler kept for API compatibility
void IRAM_ATTR c_OutputRmt::ISR_Handler(uint32_t isrFlags)
{
    (void)isrFlags;
} // ISR_Handler

//----------------------------------------------------------------------------
// ISR_MoreDataToSend - delegate to configured source
inline bool IRAM_ATTR c_OutputRmt::ISR_MoreDataToSend()
{
#if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
    if (nullptr != OutputRmtConfig.pPixelDataSource)
    {
        return OutputRmtConfig.pPixelDataSource->ISR_MoreDataToSend();
    }
    else
    {
        return OutputRmtConfig.pSerialDataSource->ISR_MoreDataToSend();
    }
#else
    return OutputRmtConfig.pPixelDataSource->ISR_MoreDataToSend();
#endif // defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
} // ISR_MoreDataToSend

//----------------------------------------------------------------------------
// ISR_ResetRmtBlockPointers
inline void IRAM_ATTR c_OutputRmt::ISR_ResetRmtBlockPointers()
{
    // No hardware pointer manipulation: just reset the software ring indices
    RmtBufferWriteIndex = 0;
    SendBufferWriteIndex = 0;
    SendBufferReadIndex = 0;
    NumUsedEntriesInSendBuffer = 0;
}

//----------------------------------------------------------------------------
// ISR_StartNewDataFrame - call source StartNewFrame
inline void IRAM_ATTR c_OutputRmt::ISR_StartNewDataFrame()
{
    if (nullptr != OutputRmtConfig.pPixelDataSource)
    {
        OutputRmtConfig.pPixelDataSource->StartNewFrame();
    }
#if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
    else
    {
        OutputRmtConfig.pSerialDataSource->StartNewFrame();
    }
#endif // defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
} // StartNewDataFrame

//----------------------------------------------------------------------------
// ISR_TransferIntensityDataToRMT (no-op for software buffer approach)
void IRAM_ATTR c_OutputRmt::ISR_TransferIntensityDataToRMT(uint32_t MaxNumEntriesToTransfer)
{
    (void)MaxNumEntriesToTransfer;
} // ISR_TransferIntensityDataToRMT

//----------------------------------------------------------------------------
// ISR_WriteToBuffer - safe ring-buffer write
inline void IRAM_ATTR c_OutputRmt::ISR_WriteToBuffer(uint32_t value)
{
    // Schütze gegen Überlauf: behalte Platz für Terminator (-1) und eine kleine Reserve
    if (NumUsedEntriesInSendBuffer >= (NUM_RMT_SLOTS - 2))
    {
        ++NumRmtSlotOverruns;
        // Verwerfen ist besser als Überschreiben. Hinweis setzen für Debug.
        return;
    }

    SendBuffer[SendBufferWriteIndex].val = value;
    SendBufferWriteIndex = (SendBufferWriteIndex + 1) & (NUM_RMT_SLOTS - 1);
    ++NumUsedEntriesInSendBuffer;
}

//----------------------------------------------------------------------------
// PauseOutput
void c_OutputRmt::PauseOutput(bool PauseOutput)
{
    if (OutputIsPaused == PauseOutput)
    {
        // no change
    }
    else if (PauseOutput)
    {
        // no-op: using function API, no direct interrupt disabling here
    }

    OutputIsPaused = PauseOutput;
} // PauseOutput

//----------------------------------------------------------------------------
// StartNewFrame - build frame, copy to heap and call rmt_write_items (blocking)
bool c_OutputRmt::StartNewFrame()
{
    bool ok = true;

    do {
        if (OutputIsPaused)
            break;

        ISR_ResetRmtBlockPointers();

        //
        // 1) Interframe-Gap (RESET vom vorherigen Frame)
        //
        for (uint32_t i = 0; i < OutputRmtConfig.NumIdleBits; i++)
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID].val);

        //
        // 2) Frame-Start Bits
        //
        for (uint32_t i = 0; i < OutputRmtConfig.NumFrameStartBits; i++)
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_STARTBIT_ID].val);

        //
        // 3) Start Frame in Pixel/Serial Source
        //
        ISR_StartNewDataFrame();
        ThereIsDataToSend = ISR_MoreDataToSend();

        //
        // 4) Pixel-Bits generieren (fills SendBuffer)
        //
        ISR_CreateIntensityData();

        //
        // 5) Optional EndOfFrameBits (user configured marker)
        //
        if (OutputRmtConfig.SendEndOfFrameBits)
        {
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_END_OF_FRAME].val);
        }

        //
        // 6) FrameStopBits (WS2812 Resetzeit!)
        //
        for (uint32_t i = 0; i < OutputRmtConfig.NumFrameStopBits; i++)
        {
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID].val);
        }

        //
        // 7) Copy software ring buffer to contiguous vector
        //
        std::vector<rmt_item32_t> items;
        items.reserve(NumUsedEntriesInSendBuffer + 8);

        while (NumUsedEntriesInSendBuffer)
        {
            items.push_back(SendBuffer[SendBufferReadIndex]);
            SendBufferReadIndex = (SendBufferReadIndex + 1) & (NUM_RMT_SLOTS - 1);
            NumUsedEntriesInSendBuffer--;
        }

        // Terminator
        rmt_item32_t endItem;
        endItem.val = 0;
        items.push_back(endItem);

        // Logging
        logcon(String("[RMT] Channel ") +
            String((int)OutputRmtConfig.RmtChannelId) +
            " sending " + String(items.size()) +
            " RMT items (" + String(float(items.size()) / 24.0, 1) + " pixelbits approx.)");

        if (items.size() < 24)
        {
            logcon(String("[RMT] WARNING: only ") + String(items.size()) +
                   " items generated -> check ISR_MoreDataToSend()");
        }

        // Copy into heap memory because rmt_write_items may use the buffer synchronously/asynchronously.
        size_t count = items.size();
        rmt_item32_t* heap_items = (rmt_item32_t*)malloc(count * sizeof(rmt_item32_t));
        if (!heap_items)
        {
            logcon("[RMT] ERROR: malloc failed for RMT items");
            ok = false;
            break;
        }
        memcpy(heap_items, items.data(), count * sizeof(rmt_item32_t));

        // BLOCKIEREND senden (stabil)
        esp_err_t e = rmt_write_items(
            (rmt_channel_t)OutputRmtConfig.RmtChannelId,
            heap_items,
            count,
            true
        );

        free(heap_items);

        if (e != ESP_OK)
        {
            logcon("[RMT] ERROR rmt_write_items failed");
            ok = false;
            break;
        }

    } while (false);

    return ok;
} // StartNewFrame

#endif // def ARDUINO_ARCH_ESP32
