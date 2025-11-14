/*
* OutputRmt.cpp - driver code for ESPixelStick RMT Channel
*
* Project: ESPixelStick - An ESP8266 / ESP32 and E1.31 based pixel driver
* Copyright (c) 2015, 2024 Shelby Merrick
* http://www.forkineye.com
*
* Adapted to use IDF4-style RMT function API (driver/rmt.h) so it compiles with
* platform-espressif32 @ 6.12.0 and runs on ESP32-S3 without direct register access.
*
* UPDATED: Non-blocking RMT output with watcher task and reduced realloc jitter.
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
static uint32_t FrameCompletes = 0;
static uint32_t FrameTimeouts = 0;

// small struct passed to watcher task
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

    // wait until the RMT transmitter finishes for this channel (blocking in watcher task)
    rmt_wait_tx_done((rmt_channel_t)ch, portMAX_DELAY);

    // notify the sendframe task that the transmission finished (to mimic original behavior)
    if (SendFrameTaskHandle)
    {
        // We're in a task context, so use the task API (not the ISR variant)
        xTaskNotifyGive(SendFrameTaskHandle);
    }

    // free the items memory
    free(p->items);
    free(p);
    vTaskDelete(NULL);
}

//----------------------------------------------------------------------------
// Simple send task that polls all channels and triggers StartNextFrame()
void RMT_Task(void *arg)
{
    (void)arg;
    while (1)
    {
        // yield a bit to other tasks but allow low latency
        vTaskDelay(pdMS_TO_TICKS(1));

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
                    // use an indefinite wait to avoid spurious timeouts
                    uint32_t NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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

            UpdateBitXlatTable(OutputRmtConfig.CitrdsArray);
            bool ok = ValidateBitXlatTable(OutputRmtConfig.CitrdsArray);

            #if defined(rmt_set_gpio)
                rmt_set_gpio(ch, RMT_MODE_TX, (gpio_num_t)OutputRmtConfig.DataPin, true);
            #endif
        }

        #if defined(SOC_RMT_SUPPORT_REF_TICK)
            ESP_ERROR_CHECK(rmt_set_source_clk(OutputRmtConfig.RmtChannelId, RMT_BASECLK_APB));
        #endif

        // reset the internal indices & buffer counters (kept for compatibility; not used)
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
// (Legacy) ISR_CreateIntensityData kept for API compatibility but not used by StartNewFrame
void IRAM_ATTR c_OutputRmt::ISR_CreateIntensityData()
{
    (void)0;
}

//----------------------------------------------------------------------------
// ISR_GetNextIntensityToSend - delegate to pixel or serial source (kept)
inline bool IRAM_ATTR c_OutputRmt::ISR_GetNextIntensityToSend(uint32_t &DataToSend)
{
    if (nullptr != OutputRmtConfig.pPixelDataSource)
    {
        bool more = OutputRmtConfig.pPixelDataSource->ISR_GetNextIntensityToSend(DataToSend);
        return more;
    }
#if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
    else
    {
        bool more = OutputRmtConfig.pSerialDataSource->ISR_GetNextIntensityToSend(DataToSend);
        return more;
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
// ISR_ResetRmtBlockPointers (legacy; no ring buffer used anymore)
inline void IRAM_ATTR c_OutputRmt::ISR_ResetRmtBlockPointers()
{
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
// ISR_WriteToBuffer kept for compatibility (writes into legacy SendBuffer)
inline void IRAM_ATTR c_OutputRmt::ISR_WriteToBuffer(uint32_t value)
{
    if (NumUsedEntriesInSendBuffer >= (NUM_RMT_SLOTS - 2))
    {
        ++NumRmtSlotOverruns;
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
// StartNewFrame - build frame in a linear vector and call rmt_write_items (non-blocking)
bool c_OutputRmt::StartNewFrame()
{
    bool ok = true;

    do {
        if (OutputIsPaused)
            break;

        // Debug start
        #ifdef USE_RMT_DEBUG_COUNTERS
            FrameStartCounter++;
            IntensityValuesSentLastFrame = IntensityValuesSent;
            IntensityValuesSent = 0;
            IntensityBitsSentLastFrame = IntensityBitsSent;
            IntensityBitsSent = 0;
        #endif

        // Ask source to start a new frame
        ISR_StartNewDataFrame();

        // Prepare linear container for rmt items for this frame
        std::vector<rmt_item32_t> items;
        // reserve a larger initial capacity to avoid repeated reallocations for typical strips
        // Dynamische Reservierung basierend auf Pixelanzahl
        uint32_t est_items = 0;
        if (OutputRmtConfig.pPixelDataSource) {
            uint32_t numPixels = OutputRmtConfig.pPixelDataSource->GetNumOutputBufferPixels();
            uint32_t bitsPerPixel = OutputRmtConfig.IntensityDataWidth;
            est_items = numPixels * bitsPerPixel;
        }
        // Frame-Overhead hinzuaddieren
        est_items += OutputRmtConfig.NumIdleBits
                   + OutputRmtConfig.NumFrameStartBits
                   + OutputRmtConfig.NumFrameStopBits
                   + 64; // Sicherheitsreserve

        // Mindestgröße
        if (est_items < 256) est_items = 256;
        items.reserve(est_items);

        // 1) Inter-frame gap (idle bits)
        for (uint32_t i = 0; i < OutputRmtConfig.NumIdleBits; ++i)
        {
            items.push_back(Intensity2Rmt[RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID]);
        }

        // 2) Frame start bits
        for (uint32_t i = 0; i < OutputRmtConfig.NumFrameStartBits; ++i)
        {
            items.push_back(Intensity2Rmt[RmtDataBitIdType_t::RMT_STARTBIT_ID]);
        }

        // 3) Pull intensity bytes from the configured source and convert to RMT items
        bool more = ISR_MoreDataToSend();
        while (more)
        {
            uint32_t intensityByte = 0;
            // Get next intensity (returns whether there will be more AFTER this byte)
            bool moreAfter = ISR_GetNextIntensityToSend(intensityByte);

            // Convert this byte into bit items according to DataDirection and IntensityDataWidth
            uint32_t mask;
            if (OutputRmtConfig.DataDirection == OutputRmtConfig_t::DataDirection_t::MSB2LSB)
            {
                mask = 1u << (OutputRmtConfig.IntensityDataWidth - 1);
            }
            else
            {
                mask = 1u;
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

                items.push_back(isOne
                    ? Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID]
                    : Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID]);

                #ifdef USE_RMT_DEBUG_COUNTERS
                    IntensityBitsSent++;
                #endif
            }

            // optional inter-intensity bit
            if (OutputRmtConfig.SendInterIntensityBits)
            {
                items.push_back(Intensity2Rmt[RmtDataBitIdType_t::RMT_STOPBIT_ID]);
            }

            // book-keeping
            #ifdef USE_RMT_DEBUG_COUNTERS
                IntensityValuesSent++;
            #endif

            // prepare next iteration
            more = moreAfter;
            if (!more)
            {
                // If source said no more, optionally append end-of-frame marker
                if (OutputRmtConfig.SendEndOfFrameBits)
                {
                    items.push_back(Intensity2Rmt[RmtDataBitIdType_t::RMT_END_OF_FRAME]);
                }
                break;
            }

            // small safety: vector will grow as needed
        } // while more data

        // 4) Frame stop bits (reset time)
        for (uint32_t i = 0; i < OutputRmtConfig.NumFrameStopBits; ++i)
        {
            items.push_back(Intensity2Rmt[RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID]);
        }

        // Terminator item to keep behavior similar to prior code
        rmt_item32_t endItem;
        endItem.val = 0;
        items.push_back(endItem);

        if (items.size() < 24)
        {
            logcon(String("[RMT] WARNING: only ") + String(items.size()) +
                " items generated -> check ISR_MoreDataToSend()");
        }

        // Copy into heap memory because rmt_write_items will be non-blocking.
        size_t count = items.size();
        rmt_item32_t* heap_items = (rmt_item32_t*)malloc(count * sizeof(rmt_item32_t));
        if (!heap_items)
        {
            logcon("[RMT] ERROR: malloc failed for RMT items");
            ok = false;
            break;
        }
        memcpy(heap_items, items.data(), count * sizeof(rmt_item32_t));

        // NON-BLOCKING send - allow multiple channels to run in parallel and avoid frame-level blocking
        esp_err_t e = rmt_write_items(
            (rmt_channel_t)OutputRmtConfig.RmtChannelId,
            heap_items,
            count,
            false // non-blocking
        );

        if (e != ESP_OK)
        {
            logcon("[RMT] ERROR rmt_write_items failed");
            free(heap_items);
            ok = false;
            break;
        }

        // spawn watcher task to free heap items and notify main send task when done
        TransmitWatcherParam * param = (TransmitWatcherParam*)malloc(sizeof(TransmitWatcherParam));
        if (!param)
        {
            // if we can't allocate a watcher, free memory and treat as error
            free(heap_items);
            logcon("[RMT] ERROR: malloc failed for TransmitWatcherParam");
            ok = false;
            break;
        }
        param->channel = (int)OutputRmtConfig.RmtChannelId;
        param->items = heap_items;
        param->count = count;

        // create watcher task - let it run on the same core as RMT_Task to avoid cross-core issues
        BaseType_t rc = xTaskCreatePinnedToCore(TransmitWatcherTask, "RMTWatcher", 2048, param, 5, NULL, 1);
        if (rc != pdPASS)
        {
            // if we couldn't create a watcher task, clean up and report error
            free(heap_items);
            free(param);
            logcon("[RMT] ERROR: failed to create TransmitWatcherTask");
            ok = false;
            break;
        }

    } while (false);

    return ok;
} // StartNewFrame

#endif // def ARDUINO_ARCH_ESP32
