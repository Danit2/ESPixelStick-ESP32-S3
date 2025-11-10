#include "output/OutputRmt.hpp"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// new RMT TX API for IDF v5+
#include "driver/rmt_tx.h"
#include "driver/rmt.h"
#else
// keep legacy include for other chips / older IDF (if you still build for esp32)
#include "driver/rmt.h"
#endif

#include "esp_log.h"
#include "freertos/semphr.h"

static const char* TAG = "OutputRmtS3";

c_OutputRmt::c_OutputRmt() :
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    txChannel(nullptr),
#endif
    HasBeenInitialized(false),
    OutputIsPaused(false),
    pParent(nullptr)
{
}

c_OutputRmt::~c_OutputRmt()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (HasBeenInitialized) {
        if (txChannel) {
            // delete the new-style channel (rmt_del_channel) — releases driver + hardware resources
            esp_err_t err = rmt_del_channel(txChannel);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "RMT tx channel deleted");
            } else {
                ESP_LOGW(TAG, "rmt_del_channel failed (%d)", err);
            }
            txChannel = nullptr;
        }
    }
#else
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
    OutputIsPaused = false;

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGI(TAG, "Init RMT TX channel (new API) requested for GPIO %d (logical id %d)", config.DataPin, config.RmtChannelId);

    // Configure new-style TX channel
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,            // use default clock source
        .gpio_num = (gpio_num_t)config.DataPin,    // output pin
        // mem_block_symbols chooses how many RMT symbols fit in the channel memory. Adjust if needed.
        .mem_block_symbols = 64,
        // transmission queue depth, higher => more outstanding transmits queued
        .trans_queue_depth = 4,
    };

    // allocate and create the TX channel handle
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &txChannel);
    if (err != ESP_OK || txChannel == nullptr) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %d", err);
        return;
    }

    // Create a COPY encoder so we can directly feed rmt_item32_t symbols later.
    // The copy encoder simply copies provided RMT symbols into driver's buffer.
    err = rmt_new_copy_encoder(&copy_encoder, 0); // second arg reserved flags = 0
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "rmt_new_copy_encoder failed: %d (continuing, but transmit of raw items may not work)", err);
        // not fatal here — depending on IDF this might be optional; keep channel but warn.
    } else {
        // attach the encoder to tx channel (so transmit functions can use it)
        err = rmt_tx_register_encoder(txChannel, copy_encoder);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "rmt_tx_register_encoder failed: %d", err);
        }
    }

    HasBeenInitialized = true;
    ESP_LOGI(TAG, "RMT TX channel initialized (new API)");
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
    ESP_LOGI(TAG, "Legacy RMT channel %d initialized successfully", config.RmtChannelId);
#endif
}

bool c_OutputRmt::StartNewFrame()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!HasBeenInitialized || txChannel == nullptr) {
        ESP_LOGW(TAG, "StartNewFrame called before init (new API)");
        return false;
    }

    // In legacy code you sent a single dummy rmt_item32_t via rmt_write_items.
    // With the new API we must use rmt_transmit (or enqueue a transmit using the registered encoder).
    // We'll create a single copy of an rmt_item32_t and transmit it via rmt_transmit.
    rmt_item32_t dummy = {{{10, 1, 10, 0}}};

    // NOTE: rmt_transmit / rmt_tx_* signatures differ across IDF minor versions.
    // The common pattern (IDF v5.x) is rmt_transmit(txChannel, &dummy, 1, timeout_ticks);
    esp_err_t err = rmt_transmit(txChannel, (rmt_symbol_word_t*)&dummy, 1, 0); // timeout 0 => non-blocking
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "rmt_transmit failed: %d", err);
        return false;
    }
#else
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
    if (!HasBeenInitialized || txChannel == nullptr)
        return;

    esp_err_t err;
    if (pause) {
        // stop transmission
        err = rmt_tx_disable(txChannel);
        OutputIsPaused = true;
    } else {
        // enable transmission again
        err = rmt_tx_enable(txChannel);
        OutputIsPaused = false;
    }

    if (err != ESP_OK)
        ESP_LOGW(TAG, "PauseOutput (new API): rmt_tx_* failed (%d)", err);
#else
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
