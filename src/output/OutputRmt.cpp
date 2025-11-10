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
static void IRAM_ATTR   rmt_intr_handler (void* param) { (void)param; }

// keep an array of pointers similar to original design
static c_OutputRmt *    rmt_isr_ThisPtrs[MAX_NUM_RMT_CHANNELS] = { nullptr };

#ifdef USE_RMT_DEBUG_COUNTERS
static uint32_t RawIsrCounter = 0;
#endif // def USE_RMT_DEBUG_COUNTERS

static TaskHandle_t SendFrameTaskHandle = NULL;
static BaseType_t xHigherPriorityTaskWoken = pdTRUE;
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
void RMT_Task (void *arg)
{
    (void)arg;
    while(1)
    {
        // Give the outputs a chance to catch up.
        delay(1);
        // process all possible channels
        for (c_OutputRmt * pRmt : rmt_isr_ThisPtrs)
        {
            // do we have a driver on this channel?
            if(nullptr != pRmt)
            {
                // invoke the channel
                if (pRmt->StartNextFrame())
                {
                    // wait for notification that the frame finished (emulates old behavior)
                    uint32_t NotificationValue = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(100) );
                    if(1 == NotificationValue)
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
c_OutputRmt::c_OutputRmt()
{
    memset((void *)&Intensity2Rmt[0], 0x00, sizeof(Intensity2Rmt));
    memset((void *)&SendBuffer[0],    0x00, sizeof(SendBuffer));

#ifdef USE_RMT_DEBUG_COUNTERS
    memset((void *)&BitTypeCounters[0], 0x00, sizeof(BitTypeCounters));
#endif // def USE_RMT_DEBUG_COUNTERS
} // c_OutputRmt

//----------------------------------------------------------------------------
c_OutputRmt::~c_OutputRmt ()
{
    if (HasBeenInitialized)
    {
        String Reason = (F("Shutting down an RMT channel requires a reboot"));
        RequestReboot(Reason, 100000);

        int ch = OutputRmtConfig.RmtChannelId;
        // disable channel and driver for that channel
        rmt_set_gpio((rmt_channel_t)ch, RMT_MODE_TX, (gpio_num_t)OutputRmtConfig.DataPin, false);
        rmt_driver_uninstall((rmt_channel_t)ch);

        rmt_isr_ThisPtrs[OutputRmtConfig.RmtChannelId] = (c_OutputRmt*)nullptr;
    }
} // ~c_OutputRmt

//----------------------------------------------------------------------------
void c_OutputRmt::Begin (OutputRmtConfig_t config, c_OutputCommon * _pParent )
{
    do // once
    {
        if(HasBeenInitialized)
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
#endif // defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
        {
            String Reason = (F("Invalid RMT configuration parameters. Rebooting"));
            RequestReboot(Reason, 10000);
            break;
        }

        NumRmtSlotsPerIntensityValue = OutputRmtConfig.IntensityDataWidth + ((OutputRmtConfig.SendInterIntensityBits) ? 1 : 0);
        if(OutputRmtConfig_t::DataDirection_t::MSB2LSB == OutputRmtConfig.DataDirection)
        {
            TxIntensityDataStartingMask = 1 << (OutputRmtConfig.IntensityDataWidth - 1);
        }
        else
        {
            TxIntensityDataStartingMask = 1;
        }

        // Configure RMT channel using IDF4-style API
        rmt_config_t RmtConfig;
        memset(&RmtConfig, 0, sizeof(RmtConfig));
        RmtConfig.rmt_mode = rmt_mode_t::RMT_MODE_TX;
        RmtConfig.channel = (rmt_channel_t)OutputRmtConfig.RmtChannelId;
        RmtConfig.gpio_num = (gpio_num_t)OutputRmtConfig.DataPin;
        RmtConfig.clk_div = 1; // feine Auflösung: 12.5 ns ticks
        RmtConfig.mem_block_num = 1;
        RmtConfig.tx_config.carrier_freq_hz = 10; // avoid zero due to driver bug
        RmtConfig.tx_config.carrier_level = rmt_carrier_level_t::RMT_CARRIER_LEVEL_LOW;
        RmtConfig.tx_config.carrier_duty_percent = 50;
        RmtConfig.tx_config.idle_level = OutputRmtConfig.idle_level;
        RmtConfig.tx_config.carrier_en = false;
        RmtConfig.tx_config.loop_en = true;
        RmtConfig.tx_config.idle_output_en = true;

        ResetGpio(OutputRmtConfig.DataPin);
        ESP_ERROR_CHECK(rmt_config(&RmtConfig));

        // install driver for that channel (channel mask uses 1<<channel)
        uint32_t ch_mask = (1u << OutputRmtConfig.RmtChannelId);
        ESP_ERROR_CHECK(rmt_driver_install((rmt_channel_t)OutputRmtConfig.RmtChannelId, 0, 0));

        // NOTE: we are NOT using direct register-based ISR anymore. The code will start
        // asynchronous writes and use a watcher task to wait for completion and notify.

        // reset the internal indices & buffer counters
        ISR_ResetRmtBlockPointers ();
        memset((void*)&SendBuffer[0], 0x0, sizeof(SendBuffer));

        UpdateBitXlatTable(OutputRmtConfig.CitrdsArray);

        if(!SendFrameTaskHandle)
        {
            xTaskCreatePinnedToCore(RMT_Task, "RMT_Task", 4096, NULL, 5, &SendFrameTaskHandle, 1);
            vTaskPrioritySet(SendFrameTaskHandle, 5);
        }
        pParent = _pParent;
        rmt_isr_ThisPtrs[OutputRmtConfig.RmtChannelId] = this;

        HasBeenInitialized = true;
    } while (false);
} // Begin

//----------------------------------------------------------------------------
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

//----------------------------------------------------------------------------
bool c_OutputRmt::ValidateBitXlatTable(const CitrdsArray_t * CitrdsArray)
{
    bool Response = false;
    if (nullptr != OutputRmtConfig.CitrdsArray)
    {
        const ConvertIntensityToRmtDataStreamEntry_t *CurrentTranslation = CitrdsArray;
        while (CurrentTranslation->Id != RmtDataBitIdType_t::RMT_LIST_END)
        {
            SetIntensity2Rmt(CurrentTranslation->Translation, CurrentTranslation->Id);

            if(Intensity2Rmt[CurrentTranslation->Id].val != CurrentTranslation->Translation.val)
            {
                logcon(String(CN_stars) + F("ERROR: incorrect bit translation deteced. Chan: ") + String(OutputRmtConfig.RmtChannelId) +
                        F(" Slot: ") + String(CurrentTranslation->Id) +
                        F(" Got: 0x") + String(Intensity2Rmt[CurrentTranslation->Id].val, HEX) +
                        F(" Expected: 0x") + String(CurrentTranslation->Translation.val));
            }

            CurrentTranslation++;
        }
    }
    else
    {
        logcon(String(CN_stars) + F("ERROR: Missing pointer to RMT bit translation values (2)") + CN_stars);
    }
    return Response;
} // ValidateBitXlatTable

//----------------------------------------------------------------------------
void c_OutputRmt::GetStatus (ArduinoJson::JsonObject& jsonStatus)
{
    jsonStatus[F("NumRmtSlotOverruns")] = NumRmtSlotOverruns;
#ifdef USE_RMT_DEBUG_COUNTERS
    jsonStatus[F("OutputIsPaused")] = OutputIsPaused;
    JsonObject debugStatus = jsonStatus["RMT Debug"].to<JsonObject>();
    debugStatus["RmtChannelId"]                 = OutputRmtConfig.RmtChannelId;
    debugStatus["GPIO"]                         = OutputRmtConfig.DataPin;

    // register-based fields are no longer available without direct register access,
    // so mark them as N/A to avoid compile errors and keep the JSON schema stable.
    debugStatus["conf0"]                        = String("N/A");
    debugStatus["conf1"]                        = String("N/A");
    debugStatus["tx_lim_ch"]                    = String("N/A");

    debugStatus["ErrorIsr"]                     = ErrorIsr;
    debugStatus["FrameCompletes"]               = String (FrameCompletes);
    debugStatus["FrameStartCounter"]            = FrameStartCounter;
    debugStatus["FrameTimeouts"]                = String (FrameTimeouts);
    debugStatus["FailedToSendAllData"]          = String (FailedToSendAllData);
    debugStatus["IncompleteFrame"]              = IncompleteFrame;
    debugStatus["IntensityValuesSent"]          = IntensityValuesSent;
    debugStatus["IntensityValuesSentLastFrame"] = IntensityValuesSentLastFrame;
    debugStatus["IntensityBitsSent"]            = IntensityBitsSent;
    debugStatus["IntensityBitsSentLastFrame"]   = IntensityBitsSentLastFrame;
    debugStatus["IntTxEndIsrCounter"]           = IntTxEndIsrCounter;
    debugStatus["IntTxThrIsrCounter"]           = IntTxThrIsrCounter;
    debugStatus["ISRcounter"]                   = ISRcounter;
    debugStatus["NumIdleBits"]                  = OutputRmtConfig.NumIdleBits;
    debugStatus["NumFrameStartBits"]            = OutputRmtConfig.NumFrameStartBits;
    debugStatus["NumFrameStopBits"]             = OutputRmtConfig.NumFrameStopBits;
    debugStatus["NumRmtSlotsPerIntensityValue"] = NumRmtSlotsPerIntensityValue;
    debugStatus["OneBitValue"]                  = String (Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID].val,  HEX);
    debugStatus["RanOutOfData"]                 = RanOutOfData;
    debugStatus["RawIsrCounter"]                = RawIsrCounter;
    debugStatus["RmtEntriesTransfered"]         = RmtEntriesTransfered;
    debugStatus["RmtWhiteDetected"]             = String (RmtWhiteDetected);
    debugStatus["RmtXmtFills"]                  = RmtXmtFills;
    debugStatus["RxIsr"]                        = RxIsr;
    debugStatus["SendBlockIsrCounter"]          = SendBlockIsrCounter;
    debugStatus["SendInterIntensityBits"]       = OutputRmtConfig.SendInterIntensityBits;
    debugStatus["SendEndOfFrameBits"]           = OutputRmtConfig.SendEndOfFrameBits;
    debugStatus["TX END int_ena"]               = "N/A";
    debugStatus["TX END int_st"]                = "N/A";
    debugStatus["TX THRSH int_ena"]             = "N/A";
    debugStatus["TX THRSH int_st"]              = "N/A";
    debugStatus["UnknownISRcounter"]            = UnknownISRcounter;
    debugStatus["ZeroBitValue"]                 = String (Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID].val, HEX);
#endif // def USE_RMT_DEBUG_COUNTERS
} // GetStatus

//----------------------------------------------------------------------------
void IRAM_ATTR c_OutputRmt::ISR_CreateIntensityData ()
{
    register uint32_t OneBitValue  = Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID].val;
    register uint32_t ZeroBitValue = Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID].val;

    uint32_t IntensityValue; // = 0;
    uint32_t NumAvailableBufferSlotsToFill = NUM_RMT_SLOTS - NumUsedEntriesInSendBuffer;
    while ((NumAvailableBufferSlotsToFill > NumRmtSlotsPerIntensityValue) && ThereIsDataToSend)
    {
        ThereIsDataToSend = ISR_GetNextIntensityToSend(IntensityValue);
        RMT_DEBUG_COUNTER(IntensityValuesSent++);
#ifdef USE_RMT_DEBUG_COUNTERS
        if(200 < IntensityValue)
        {
            ++RmtWhiteDetected;
        }
#endif // def USE_RMT_DEBUG_COUNTERS

        // convert the intensity data into RMT slot data
        uint32_t bitmask = TxIntensityDataStartingMask;
        for (uint32_t BitCount = OutputRmtConfig.IntensityDataWidth; 0 < BitCount; --BitCount)
        {
            RMT_DEBUG_COUNTER(IntensityBitsSent++);
            ISR_WriteToBuffer((IntensityValue & bitmask) ? OneBitValue : ZeroBitValue);
            if(OutputRmtConfig_t::DataDirection_t::MSB2LSB == OutputRmtConfig.DataDirection)
            {
                bitmask >>= 1;
            }
            else
            {
                bitmask <<= 1;
            }
#ifdef USE_RMT_DEBUG_COUNTERS
            if (IntensityValue & bitmask)
            {
                BitTypeCounters[int(RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID)]++;
            }
            else
            {
                BitTypeCounters[int(RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID)]++;
            }
#endif // def USE_RMT_DEBUG_COUNTERS
        } // end send one intensity value

        if (OutputRmtConfig.SendEndOfFrameBits && !ThereIsDataToSend)
        {
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_END_OF_FRAME].val);
            RMT_DEBUG_COUNTER(BitTypeCounters[int(RmtDataBitIdType_t::RMT_END_OF_FRAME)]++);
        }
        else if (OutputRmtConfig.SendInterIntensityBits)
        {
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_STOP_START_BIT_ID].val);
            RMT_DEBUG_COUNTER(BitTypeCounters[int(RmtDataBitIdType_t::RMT_STOP_START_BIT_ID)]++);
        }

        // recalc how much space is still in the buffer.
        NumAvailableBufferSlotsToFill = (NUM_RMT_SLOTS - 1) - NumUsedEntriesInSendBuffer;
    } // end while there is space in the buffer
} // ISR_Handler_SendIntensityData

//----------------------------------------------------------------------------
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
} // GetNextIntensityToSend

//----------------------------------------------------------------------------
void IRAM_ATTR c_OutputRmt::ISR_Handler (uint32_t isrFlags)
{
    // ISR_Handler is retained for API compatibility, but we do not use register-level IRQ handling.
    (void)isrFlags;
} // ISR_Handler

//----------------------------------------------------------------------------
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
}

//----------------------------------------------------------------------------
inline void IRAM_ATTR c_OutputRmt::ISR_ResetRmtBlockPointers()
{
    // No hardware pointer manipulation: just reset the software ring indices
    RmtBufferWriteIndex = 0;
    SendBufferWriteIndex = 0;
    SendBufferReadIndex  = 0;
    NumUsedEntriesInSendBuffer = 0;
}

//----------------------------------------------------------------------------
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
void IRAM_ATTR c_OutputRmt::ISR_TransferIntensityDataToRMT (uint32_t MaxNumEntriesToTransfer)
{
    // In the old register-based code this copied into RMTMEM.chan[].data32.
    // Here we just keep data in the software ring buffer (SendBuffer). The actual transfer
    // to the hardware happens at StartNewFrame by copying into a contiguous block and calling rmt_write_items.
    (void)MaxNumEntriesToTransfer;
    // nothing to do here except bookkeeping (already handled by ISR_WriteToBuffer)
} // ISR_Handler_TransferBufferToRMT

//----------------------------------------------------------------------------
inline void IRAM_ATTR c_OutputRmt::ISR_WriteToBuffer(uint32_t value)
{
    SendBuffer[SendBufferWriteIndex++].val = value;
    SendBufferWriteIndex &= uint32_t(NUM_RMT_SLOTS - 1);
    ++NumUsedEntriesInSendBuffer;
}

//----------------------------------------------------------------------------
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
bool c_OutputRmt::StartNewFrame ()
{
    bool Response = false;

    do // once
    {
        if(OutputIsPaused)
        {
            break;
        }

        if(InterrupsAreEnabled)
        {
            RMT_DEBUG_COUNTER(IncompleteFrame++);
        }

        // reset our indices
        ISR_ResetRmtBlockPointers ();

        // build inter-frame gap
        uint32_t NumInterFrameRmtSlotsCount = 0;
        while (NumInterFrameRmtSlotsCount < OutputRmtConfig.NumIdleBits)
        {
            ISR_WriteToBuffer (Intensity2Rmt[RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID].val);
            ++NumInterFrameRmtSlotsCount;
            RMT_DEBUG_COUNTER(BitTypeCounters[int(RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID)]++);
        }

        // frame start bits
        uint32_t NumFrameStartRmtSlotsCount = 0;
        while (NumFrameStartRmtSlotsCount++ < OutputRmtConfig.NumFrameStartBits)
        {
            ISR_WriteToBuffer (Intensity2Rmt[RmtDataBitIdType_t::RMT_STARTBIT_ID].val);
            RMT_DEBUG_COUNTER(BitTypeCounters[int(RmtDataBitIdType_t::RMT_STARTBIT_ID)]++);
        }

#ifdef USE_RMT_DEBUG_COUNTERS
        FrameStartCounter++;
        IntensityValuesSentLastFrame = IntensityValuesSent;
        IntensityValuesSent          = 0;
        IntensityBitsSentLastFrame   = IntensityBitsSent;
        IntensityBitsSent            = 0;
#endif // def USE_RMT_DEBUG_COUNTERS

        // set up to send a new frame
        ISR_StartNewDataFrame ();

        // create initial data
        ThereIsDataToSend = ISR_MoreDataToSend();
        ISR_CreateIntensityData();

        // refill if needed
        ISR_CreateIntensityData();

        // Collect SendBuffer entries into a contiguous array for rmt_write_items()
        std::vector<rmt_item32_t> tx_items;
        tx_items.reserve(NumUsedEntriesInSendBuffer + 8);

        // drain current buffer first, then generate more until no more data
        while(true)
        {
            // move available SendBuffer items to tx_items
            while(NumUsedEntriesInSendBuffer)
            {
                rmt_item32_t &it = SendBuffer[SendBufferReadIndex];
                tx_items.push_back(it);

                SendBufferReadIndex = (SendBufferReadIndex + 1) & (NUM_RMT_SLOTS - 1);
                --NumUsedEntriesInSendBuffer;
            }

            // if we still have data upstream, ask for more and create it
            if (ISR_MoreDataToSend())
            {
                ThereIsDataToSend = true;
                ISR_CreateIntensityData();
                // loop to drain newly created entries
            }
            else
            {
                // no more data to append
                ThereIsDataToSend = false;
                break;
            }
        }

        // ensure termination value like original (0)
        rmt_item32_t terminator;
        terminator.val = 0;
        tx_items.push_back(terminator);

        // Copy into heap memory because rmt_write_items may use the buffer asynchronously.
        size_t count = tx_items.size();
        rmt_item32_t * heap_items = (rmt_item32_t*)malloc(count * sizeof(rmt_item32_t));
        if(!heap_items)
        {
            logcon(String(CN_stars) + F(" ERROR: malloc failed for RMT items") + CN_stars);
            break;
        }
        memcpy(heap_items, tx_items.data(), count * sizeof(rmt_item32_t));

        // Start non-blocking transmit: write items and return immediately.
        int ch = OutputRmtConfig.RmtChannelId;
        esp_err_t err = rmt_write_items((rmt_channel_t)ch, heap_items, count, false); // non-blocking
        if (err != ESP_OK)
        {
            logcon(String(CN_stars) + F(" ERROR: rmt_write_items failed") + CN_stars);
            free(heap_items);
            break;
        }

        // create a watcher param to free buffer and notify when done
        TransmitWatcherParam * p = (TransmitWatcherParam*)malloc(sizeof(TransmitWatcherParam));
        if(!p)
        {
            logcon(String(CN_stars) + F(" ERROR: malloc failed for watcher param") + CN_stars);
            // If we cannot create watcher, at least wait here (blocking) and then free
            rmt_wait_tx_done((rmt_channel_t)ch, portMAX_DELAY);
            free(heap_items);
            break;
        }
        p->channel = ch;
        p->items = heap_items;
        p->count = count;

        // Wait synchronously for TX completion (no extra task → no heap use)
		rmt_wait_tx_done((rmt_channel_t)ch, portMAX_DELAY);
		free(heap_items);
		free(p);

		// notify SendFrameTaskHandle to keep timing identical
		if (SendFrameTaskHandle) {
			vTaskNotifyGiveFromISR(SendFrameTaskHandle, &xHigherPriorityTaskWoken);
		}

        Response = true;
    } while(false);

    return Response;
} // StartNewFrame

#endif // def ARDUINO_ARCH_ESP32
