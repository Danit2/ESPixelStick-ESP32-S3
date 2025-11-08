#include "output/OutputRmt.hpp"
#include "driver/rmt.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char* TAG = "OutputRmtS3";

c_OutputRmt::c_OutputRmt() = default;

c_OutputRmt::~c_OutputRmt()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (HasBeenInitialized)
    {
        esp_err_t err = rmt_driver_uninstall(OutputRmtConfig.RmtChannelId);
        if (err == ESP_OK)
            ESP_LOGI(TAG, "RMT channel %d deinitialized", OutputRmtConfig.RmtChannelId);
        else
            ESP_LOGW(TAG, "rmt_driver_uninstall failed (%d)", err);
    }
#endif
}

void c_OutputRmt::Begin(OutputRmtConfig_t config, c_OutputCommon* outputDriver)
{
    OutputRmtConfig = config;
    pParent = outputDriver;
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

    esp_err_t err = rmt_config(&rmt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_config failed: %d", err);
        return;
    }

    err = rmt_driver_install(rmt_cfg.channel, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_driver_install failed: %d", err);
        return;
    }

    HasBeenInitialized = true;
    ESP_LOGI(TAG, "RMT channel %d initialized successfully", config.RmtChannelId);
#endif
}

bool c_OutputRmt::StartNewFrame()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!HasBeenInitialized) {
        ESP_LOGW(TAG, "StartNewFrame called before init");
        return false;
    }
    rmt_item32_t dummy = {{{10, 1, 10, 0}}};
    esp_err_t err = rmt_write_items((rmt_channel_t)OutputRmtConfig.RmtChannelId, &dummy, 1, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "rmt_write_items failed: %d", err);
        return false;
    }
#endif
    return true;
}

void c_OutputRmt::PauseOutput(bool pause)
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!HasBeenInitialized)
        return;

    esp_err_t err;
    if (pause) {
        err = rmt_tx_stop((rmt_channel_t)OutputRmtConfig.RmtChannelId);
        OutputIsPaused = true;
    } else {
        err = rmt_tx_start((rmt_channel_t)OutputRmtConfig.RmtChannelId, true);
        OutputIsPaused = false;
    }

    if (err != ESP_OK)
        ESP_LOGW(TAG, "PauseOutput: rmt_tx_* failed (%d)", err);
#endif
}

void c_OutputRmt::ISR_Handler(uint32_t isrFlags)
{
    (void)isrFlags;
}

void c_OutputRmt::GetStatus(ArduinoJson::JsonObject& jsonStatus)
{
    jsonStatus["RMT_Channel"] = (int)OutputRmtConfig.RmtChannelId;
    jsonStatus["GPIO"]        = (int)OutputRmtConfig.DataPin;
    jsonStatus["Init"]        = HasBeenInitialized;
    jsonStatus["Paused"]      = OutputIsPaused;
}

bool c_OutputRmt::ValidateBitXlatTable(const CitrdsArray_t* CitrdsArray)
{
    if (!CitrdsArray)
        return false;

    uint32_t count = 0;
    const CitrdsArray_t* entry = CitrdsArray;
    while (entry->Id != RMT_LIST_END) {
        ++count;
        ++entry;
    }
    return (count >= 2);
}
