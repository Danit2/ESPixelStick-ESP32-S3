#include "output/OutputRmt.hpp"
#include "esp_log.h"
#include "freertos/semphr.h"

extern "C" {
    #include "esp_private/esp_clk.h"
    #include "driver/rmt_tx.h"
    #include "driver/rmt_encoder.h"
}

static const char* TAG = "OutputRmt";

c_OutputRmt::c_OutputRmt()
    : txChannel(nullptr),
      copy_encoder(nullptr),
      HasBeenInitialized(false),
      OutputIsPaused(false),
      pParent(nullptr)
{
}

c_OutputRmt::~c_OutputRmt()
{
    if (HasBeenInitialized && txChannel) {
        esp_err_t err = rmt_del_channel(txChannel);
        if (err == ESP_OK)
            ESP_LOGI(TAG, "RMT channel deleted");
        else
            ESP_LOGW(TAG, "rmt_del_channel failed: %d", err);
    }
}

void c_OutputRmt::Begin(OutputRmtConfig_t config, c_OutputCommon* outputDriver)
{
    OutputRmtConfig = config;
    pParent = outputDriver;
    HasBeenInitialized = false;
    OutputIsPaused = false;

    ESP_LOGI(TAG, "Init RMT (v2 API) GPIO %d", config.DataPin);

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

    err = rmt_enable(txChannel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %d", err);
        return;
    }

    HasBeenInitialized = true;
    ESP_LOGI(TAG, "RMT v2 channel initialized on GPIO %d", config.DataPin);
}

bool c_OutputRmt::StartNewFrame()
{
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
    return true;
}

void c_OutputRmt::PauseOutput(bool pause)
{
    if (!HasBeenInitialized || !txChannel)
        return;

    esp_err_t err = pause ? rmt_tx_disable(txChannel) : rmt_tx_enable(txChannel);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "PauseOutput failed: %d", err);

    OutputIsPaused = pause;
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
    if (!arr)
        return false;

    uint32_t count = 0;
    while (arr->Id != RMT_LIST_END) { ++count; ++arr; }
    return (count >= 2);
}
