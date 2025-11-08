#include "output/OutputRmt.hpp"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include <vector>

static const char* TAG = "OutputRmtS3";

c_OutputRmt::c_OutputRmt() = default;
c_OutputRmt::~c_OutputRmt()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (HasBeenInitialized)
    {
        rmt_driver_uninstall(OutputRmtConfig.RmtChannelId);
        ESP_LOGI(TAG, "RMT channel %d deinitialized", OutputRmtConfig.RmtChannelId);
    }
#endif
}

void c_OutputRmt::Begin(OutputRmtConfig_t config, c_OutputCommon* outputDriver)
{
    OutputRmtConfig = config;
    HasBeenInitialized = false;

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGI(TAG, "Init RMT channel %d on GPIO %d", config.RmtChannelId, config.DataPin);

    rmt_config_t rmt_cfg = {};
    rmt_cfg.rmt_mode = RMT_MODE_TX;
    rmt_cfg.channel = (rmt_channel_t)config.RmtChannelId;
    rmt_cfg.gpio_num = (gpio_num_t)config.DataPin;
    rmt_cfg.mem_block_num = 1;
    rmt_cfg.tx_config.loop_en = false;
    rmt_cfg.tx_config.carrier_en = false;
    rmt_cfg.tx_config.idle_output_en = true;
    rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_cfg.clk_div = 2; // 40 MHz base clock

    ESP_ERROR_CHECK(rmt_config(&rmt_cfg));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_cfg.channel, 0, 0));

    HasBeenInitialized = true;
#endif
}

bool c_OutputRmt::StartNewFrame()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!HasBeenInitialized) return false;
    // Dummy pulse send to keep driver active
    rmt_item32_t dummy = {{{10, 1, 10, 0}}};
    rmt_write_items((rmt_channel_t)OutputRmtConfig.RmtChannelId, &dummy, 1, true);
#endif
    return true;
}

void c_OutputRmt::PauseOutput(bool pause)
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!HasBeenInitialized) return;
    if (pause)
    {
        rmt_tx_stop((rmt_channel_t)OutputRmtConfig.RmtChannelId);
        OutputIsPaused = true;
    }
    else
    {
        rmt_tx_start((rmt_channel_t)OutputRmtConfig.RmtChannelId, true);
        OutputIsPaused = false;
    }
#endif
}

void c_OutputRmt::ISR_Handler(uint32_t isrFlags)
{
    // Not used on S3 — handled by driver
    (void)isrFlags;
}

// Statusreport für JSON / Debug
void c_OutputRmt::GetStatus(ArduinoJson::JsonObject& jsonStatus)
{
    jsonStatus["RMT_Channel"] = static_cast<int>(OutputRmtConfig.RmtChannelId);
    jsonStatus["GPIO"]        = static_cast<int>(OutputRmtConfig.DataPin);
    jsonStatus["Init"]        = HasBeenInitialized;
    jsonStatus["Paused"]      = OutputIsPaused;
}

// Plausibilitätscheck für Bitübersetzungstabellen
bool c_OutputRmt::ValidateBitXlatTable(const CitrdsArray_t* CitrdsArray)
{
    if (!CitrdsArray) return false;
    uint32_t count = 0;
    const CitrdsArray_t* entry = CitrdsArray;
    while (entry->Id != RMT_LIST_END)
    {
        ++count;
        ++entry;
    }
    return (count >= 2);
}
