#include "OutputRmt.hpp"
#include "driver/rmt.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char* TAG = "OutputRmt";

struct S3RmtChannel {
    rmt_channel_t channel;
    gpio_num_t gpio;
    size_t mem_block_num;
    SemaphoreHandle_t tx_done_sem;
    int tx_limit;
};

static S3RmtChannel s3RmtChannels[8];
static bool InIsr = false;

void s3_rmt_init_channel(rmt_channel_t channel, gpio_num_t gpio)
{
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = channel;
    config.gpio_num = gpio;
    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(channel, 0, 0));
    ESP_LOGI(TAG, "Initialized RMT channel %d on GPIO %d", channel, gpio);
}

void s3_rmt_deinit_channel(rmt_channel_t channel)
{
    ESP_ERROR_CHECK(rmt_driver_uninstall(channel));
}

void s3_rmt_write_items_blocking(rmt_channel_t channel, const rmt_item32_t* items, size_t num_items)
{
    ESP_ERROR_CHECK(rmt_write_items(channel, items, num_items, true));
    ESP_ERROR_CHECK(rmt_wait_tx_done(channel, pdMS_TO_TICKS(100)));
}

void s3_rmt_start_tx(rmt_channel_t channel)
{
    ESP_ERROR_CHECK(rmt_tx_start(channel, true));
}

void s3_rmt_stop_tx(rmt_channel_t channel)
{
    ESP_ERROR_CHECK(rmt_tx_stop(channel));
}

// ============================================================================
// Interrupt Handler Fix für ESP32-S3
// ============================================================================
static void IRAM_ATTR rmt_intr_handler(void* param)
{
#ifdef DEBUG_GPIO
    // digitalWrite(DEBUG_GPIO, HIGH);
#endif

    if (!InIsr)
    {
        InIsr = true;

        uint32_t isrFlags = 0; // Fix: declare outside conditional

#if defined(CONFIG_IDF_TARGET_ESP32)
        isrFlags = RMT.int_st.val;
        RMT.int_clr.val = uint32_t(-1);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
        // ESP32-S3: kein direkter Registerzugriff möglich, Treiber managt Flags
        isrFlags = 0xFFFFFFFF; // simulierte Flags, um ISR_Handler auszulösen
#else
        isrFlags = 0;
#endif

        while (0 != isrFlags)
        {
            // Hier alle registrierten RMT-Kanäle abarbeiten
            for (auto CurrentRmtChanThisPtr : rmt_isr_ThisPtrs)
            {
                if (nullptr != CurrentRmtChanThisPtr)
                {
                    CurrentRmtChanThisPtr->ISR_Handler(isrFlags);
                }
            }

#if defined(CONFIG_IDF_TARGET_ESP32)
            isrFlags = RMT.int_st.val;
            RMT.int_clr.val = uint32_t(-1);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
            // S3: Flags werden automatisch gelöscht → Schleife beenden
            isrFlags = 0;
#else
            isrFlags = 0;
#endif
        }

        InIsr = false;
    }

#ifdef DEBUG_GPIO
    // digitalWrite(DEBUG_GPIO, LOW);
#endif
}

// ============================================================================
// Beispiel-Methoden (verkürzt)
// ============================================================================
c_OutputRmt::c_OutputRmt()
{
    // Konstruktor-Setup
}

c_OutputRmt::~c_OutputRmt()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    s3_rmt_deinit_channel(OutputRmtConfig.RmtChannelId);
#endif
}

void c_OutputRmt::Begin(OutputRmtConfig_t config, c_OutputCommon* outputDriver)
{
    OutputRmtConfig = config;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    s3_rmt_init_channel((rmt_channel_t)OutputRmtConfig.RmtChannelId, (gpio_num_t)OutputRmtConfig.Pin);
#else
    // alter Code für ESP32 bleibt hier unverändert
#endif
}

bool c_OutputRmt::StartNewFrame()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    s3_rmt_start_tx((rmt_channel_t)OutputRmtConfig.RmtChannelId);
#endif
    return true;
}

void c_OutputRmt::ISR_Handler(uint32_t isrFlags)
{
    // hier ggf. LED-Sende-Interrupts abarbeiten
}

void c_OutputRmt::PauseOutput(bool pause)
{
    if (pause)
    {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        s3_rmt_stop_tx((rmt_channel_t)OutputRmtConfig.RmtChannelId);
#endif
    }
}

