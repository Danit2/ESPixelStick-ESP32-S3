/*
* OutputRmt.cpp - driver code for ESPixelStick RMT Channel
*
* Project: ESPixelStick - An ESP8266 / ESP32 and E1.31 based pixel driver
* Copyright (c) 2015, 2024 Shelby Merrick
* http://www.forkineye.com
*
*/
#include "ESPixelStick.h"
#ifdef ARDUINO_ARCH_ESP32
#include "output/OutputRmt.hpp"

// New RMT API headers (IDF v5+)
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>

#include <vector>

// forward declaration for the isr handler (unused for S3 copy-encoder flow)
static void IRAM_ATTR   rmt_intr_handler (void* param) { (void)param; /* no-op for S3 path */ }

// RMT handles for new API
static rmt_channel_handle_t g_rmt_chan_handles[MAX_NUM_RMT_CHANNELS] = { nullptr };
static rmt_encoder_handle_t g_rmt_encoders[MAX_NUM_RMT_CHANNELS] = { nullptr };

// task related unchanged
static TaskHandle_t SendFrameTaskHandle = NULL;
static BaseType_t xHigherPriorityTaskWoken = pdTRUE;
static uint32_t FrameCompletes = 0;
static uint32_t FrameTimeouts = 0;

#ifdef USE_RMT_DEBUG_COUNTERS
static uint32_t RawIsrCounter = 0;
#endif // def USE_RMT_DEBUG_COUNTERS

//----------------------------------------------------------------------------
void RMT_Task (void *arg)
{
    while(1)
    {
        delay(1);
        for (c_OutputRmt * pRmt : rmt_isr_ThisPtrs)
        {
            if(nullptr != pRmt)
            {
                if (pRmt->StartNextFrame())
                {
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

        // Clean up new API resources for this channel
        int ch = OutputRmtConfig.RmtChannelId;
        if (g_rmt_chan_handles[ch])
        {
            rmt_disable(g_rmt_chan_handles[ch]);
            rmt_del_channel(g_rmt_chan_handles[ch]);
            g_rmt_chan_handles[ch] = nullptr;
        }
        if (g_rmt_encoders[ch])
        {
            rmt_del_encoder(g_rmt_encoders[ch]);
            g_rmt_encoders[ch] = nullptr;
        }

        rmt_isr_ThisPtrs[OutputRmtConfig.RmtChannelId] = (c_OutputRmt*)nullptr;
    }
} // ~c_OutputRmt

//----------------------------------------------------------------------------
/* keep stubbed intr handler for compatibility (not used on S3 copy-encoder send path) */
static void IRAM_ATTR rmt_intr_handler (void* param)
{
    (void)param;
} // rmt_intr_handler


//----------------------------------------------------------------------------
void c_OutputRmt::Begin (OutputRmtConfig_t config, c_OutputCommon * _pParent )
{
    do // once
    {
        if(HasBeenInitialized)
        {
            ResetGpio(OutputRmtConfig.DataPin);
        }

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

        NumRmtSlotsPerIntensityValue = OutputRmtConfig.IntensityDataWidth + ((OutputRmtConfig.SendInterIntensityBits) ? 1 : 0);
        if(OutputRmtConfig_t::DataDirection_t::MSB2LSB == OutputRmtConfig.DataDirection)
        {
            TxIntensityDataStartingMask = 1 << (OutputRmtConfig.IntensityDataWidth - 1);
        }
        else
        {
            TxIntensityDataStartingMask = 1;
        }

        // Configure new RMT TX channel (ESP-IDF v5 style)
        rmt_tx_channel_config_t tx_chan_cfg;
        memset(&tx_chan_cfg, 0, sizeof(tx_chan_cfg));
        tx_chan_cfg.gpio_num = (gpio_num_t)OutputRmtConfig.DataPin;
        tx_chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        // Provide a high resolution (10 MHz) to keep fine granularity similar to original clock divisor approach
        tx_chan_cfg.resolution_hz = 10 * 1000 * 1000;
        // Use a reasonable memory block size (64 symbols) - adjust if you need more
        tx_chan_cfg.mem_block_symbols = 64;
        tx_chan_cfg.trans_queue_depth = 4;
        tx_chan_cfg.invert_out = false;

        int ch = OutputRmtConfig.RmtChannelId;
        // create tx channel for this channel id
        esp_err_t err = rmt_new_tx_channel(&tx_chan_cfg, &g_rmt_chan_handles[ch]);
        if (err != ESP_OK)
        {
            logcon(String(CN_stars) + F(" ERROR: rmt_new_tx_channel failed") + CN_stars);
            RequestReboot(F("RMT channel allocation failed"), 10000);
            break;
        }
        ESP_ERROR_CHECK(rmt_enable(g_rmt_chan_handles[ch]));

        // create copy encoder (we will feed already prepared RMT symbols)
        rmt_copy_encoder_config_t copy_enc_cfg;
        memset(&copy_enc_cfg, 0, sizeof(copy_enc_cfg));
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_enc_cfg, &g_rmt_encoders[ch]));

        // start background send task if not running
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
                        F(" Got: 0x") + String(uint32_t(Intensity2Rmt[CurrentTranslation->Id].val), HEX) +
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

    uint32_t IntensityValue;
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
    // This ISR handler used to handle register interrupts; for S3/copy-encoder approach it is not used.
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
    // With new API we don't touch hardware pointers directly; just reset our indices
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
    uint32_t NumEntriesToTransfer = min(NumUsedEntriesInSendBuffer, MaxNumEntriesToTransfer);

#ifdef USE_RMT_DEBUG_COUNTERS
    if(NumEntriesToTransfer)
    {
        ++RmtXmtFills;
        RmtEntriesTransfered = NumEntriesToTransfer;
    }
#endif // def USE_RMT_DEBUG_COUNTERS
    while(NumEntriesToTransfer)
    {
        // Previously we copied into hardware RMT memory. Now we keep them in the software ring
        // SendBuffer already contains the prepared rmt_item32_t values; the transfer to hardware will be done
        // by assembling a contiguous payload and calling rmt_transmit() from StartNewFrame().
        --NumEntriesToTransfer;
    }

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
        // stop output - no hardware interrupts to disable in new API path
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

        // At this point we have some prepared SendBuffer entries. For ESP32-S3 we will
        // collect all generated entries (stream style) into a contiguous vector of rmt_symbol_word_t
        // and call rmt_transmit() once with the whole payload (copy encoder).

        std::vector<rmt_symbol_word_t> tx_symbols;
        tx_symbols.reserve(1024); // pre-alloc a reasonable amount; grows if necessary

        // drain current buffer first, then generate more until no more data
        while(true)
        {
            // move available SendBuffer items to tx_symbols
            while(NumUsedEntriesInSendBuffer)
            {
                rmt_item32_t &it = SendBuffer[SendBufferReadIndex];
                rmt_symbol_word_t sym;
                sym.duration0 = it.duration0;
                sym.level0    = it.level0;
                sym.duration1 = it.duration1;
                sym.level1    = it.level1;
                tx_symbols.push_back(sym);

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

        // If we requested SendEndOfFrame bits earlier, they were written by ISR_CreateIntensityData.
        // Ensure we terminate with an empty symbol (duration0 = 0) as marker (original code wrote 0).
        rmt_symbol_word_t term_sym = {0, 0, 0, 0};
        tx_symbols.push_back(term_sym);

        // Transmit the built symbol array using the copy encoder
        rmt_transmit_config_t tx_cfg;
        memset(&tx_cfg, 0, sizeof(tx_cfg));
        tx_cfg.loop_count = 0;

        int ch = OutputRmtConfig.RmtChannelId;
        if (g_rmt_chan_handles[ch] == nullptr || g_rmt_encoders[ch] == nullptr)
        {
            logcon(String(CN_stars) + F(" ERROR: RMT channel/encoder not initialized") + CN_stars);
            break;
        }

        // Perform the transmit (blocking in driver until payload consumed)
        esp_err_t err = rmt_transmit(g_rmt_chan_handles[ch], g_rmt_encoders[ch],
                                     tx_symbols.data(), tx_symbols.size() * sizeof(rmt_symbol_word_t), &tx_cfg);
        if (err != ESP_OK)
        {
            logcon(String(CN_stars) + F(" ERROR: rmt_transmit failed") + CN_stars);
            // we still mark response false so caller knows
            break;
        }

        Response = true;
    } while(false);

    return Response;
} // StartNewFrame

#endif // def ARDUINO_ARCH_ESP32
