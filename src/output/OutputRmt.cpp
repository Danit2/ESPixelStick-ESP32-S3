#include "output/OutputRmt.hpp"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char* TAG = "OutputRmt";

// Prüfen, ob wir unter ESP-IDF >=5 laufen (neue RMT-API verfügbar)
#if ESP_IDF_VERSION_MAJOR >= 5
  extern "C" {
    // Arduino exportiert diese Header nicht direkt, deshalb der Pfad über esp_private
    #include "esp_private/esp_clk.h"
    #include "driver/rmt_tx.h"
    #include "driver/rmt_encoder.h"
  }
  #define HAS_RMT_V2_API 1
#else
  #include "driver/rmt.h"
  #define HAS_RMT_V2_API 0
#endif


c_OutputRmt::c_OutputRmt()
    :
#if HAS_RMT_V2_API
      txChannel(nullptr),
      copy_encoder(nullptr),
#endif
      HasBeenInitialized(false),
      OutputIsPaused(false),
      pParent(nullptr)
{
}

c_OutputRmt::~c_OutputRmt()
{
#if HAS_RMT_V2_API
    if (HasBeenInitialized && txChannel) {
        esp_err_t err = rmt_del_channel(txChannel);
        if (err == ESP_OK)
            ESP_LOGI(TAG, "RMT channel deleted");
        else
            ESP_LOGW(TAG, "rmt_del_channel failed: %d", err);
    }
#else
    if (HasBeenInitialized) {
        esp_err_t err = rmt_driver_uninstall(OutputRmtConfig.RmtChannelId);
        if (err == ESP_OK)
            ESP_LOGI(TAG, "Legacy RMT channel %d deinitialized", OutputRmtConfig.RmtChannelId);
        else
            ESP_LOGW(TAG, "rmt_driver_uninstall failed: %d", err);
    }
#endif
}

void c_OutputRmt::Begin(OutputRmtConfig_t config, c_OutputCommon* outputDriver)
{
    OutputRmtConfig = config;
    pParent = outputDriver;
    HasBeenInitialized = false;
    OutputIsPaused = false;

#if HAS_RMT_V2_API
    ESP_LOGI(TAG, "Init RMT (new API) GPIO %d", config.DataPin);

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = (gpio_num_t)config.DataPin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };

    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &txChannel);
    if (err != ESP_OK || !txChannel) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %d", err);
        return;
    }

    err = rmt_new_copy_encoder(&copy_encoder, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder failed: %d", err);
        return;
    }

    ESP_ERROR_CHECK(rmt_enable(txChannel));
    HasBeenInitialized = true;
    ESP_LOGI(TAG, "RMT v2 channel initialized on GPIO %d", config.DataPin);

#else
    ESP_LOGI(TAG, "Init legacy RMT channel %d on GPIO %d", config.RmtChannelId, config.DataPin);
    rmt_config_t rmt_cfg = {};
    rmt_cfg.rmt_mode = RMT_MODE_TX;
    rmt_cfg.channel = (rmt_channel_t)config.RmtChannelId;
    rmt_cfg.gpio_num = (gpio_num_t)config.DataPin;
    rmt_cfg.mem_block_num = 1;
    rmt_cfg.tx_config.loop_en = false;
    rmt_cfg.tx_config.carrier_en = false;
    rmt_cfg.tx_config.idle_output_en = true;
    rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_cfg.clk_div = 2;

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
#endif
}

bool c_OutputRmt::StartNewFrame()
{
#if HAS_RMT_V2_API
    if (!HasBeenInitialized || !txChannel) {
        ESP_LOGW(TAG, "StartNewFrame before init");
        return false;
    }
    rmt_item32_t symbol = {{{10, 1, 10, 0}}};
    esp_err_t err = rmt_transmit(txChannel, copy_encoder, &symbol, sizeof(symbol), NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "rmt_transmit failed: %d", err);
        return false;
    }
    rmt_tx_wait_all_done(txChannel, -1);
#else
    if (!HasBeenInitialized) return false;
    rmt_item32_t symbol = {{{10, 1, 10, 0}}};
    esp_err_t err = rmt_write_items((rmt_channel_t)OutputRmtConfig.RmtChannelId, &symbol, 1, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "rmt_write_items failed: %d", err);
        return false;
    }
#endif
    return true;
}

void c_OutputRmt::PauseOutput(bool pause)
{
#if HAS_RMT_V2_API
    if (!HasBeenInitialized || !txChannel) return;
    esp_err_t err = pause ? rmt_tx_disable(txChannel) : rmt_tx_enable(txChannel);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "PauseOutput failed: %d", err);
    OutputIsPaused = pause;
#else
    if (!HasBeenInitialized) return;
    esp_err_t err = pause
        ? rmt_tx_stop((rmt_channel_t)OutputRmtConfig.RmtChannelId)
        : rmt_tx_start((rmt_channel_t)OutputRmtConfig.RmtChannelId, true);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "PauseOutput failed: %d", err);
    OutputIsPaused = pause;
#endif
}

void c_OutputRmt::ISR_Handler(uint32_t isrFlags)
{
    (void)isrFlags;
}

void c_OutputRmt::GetStatus(ArduinoJson::JsonObject& jsonStatus)
{
    jsonStatus["GPIO"]   = (int)OutputRmtConfig.DataPin;
    jsonStatus["Init"]   = HasBeenInitialized;
    jsonStatus["Paused"] = OutputIsPaused;
}

bool c_OutputRmt::ValidateBitXlatTable(const CitrdsArray_t* arr)
{
    if (!arr) return false;
    uint32_t count = 0;
    while (arr->Id != RMT_LIST_END) { ++count; ++arr; }
    return (count >= 2);
}
