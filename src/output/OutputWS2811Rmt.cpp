#include "output/OutputWS2811Rmt.hpp"
#include "output/OutputMgr.hpp"

#if defined(SUPPORT_OutputType_WS2811) && defined(ARDUINO_ARCH_ESP32)

/*****************************************************************************************
 * WS2811 Timing definitions (auto-calculated for ESP32-S3 RMT)
 *****************************************************************************************/

// WS2811 runs at 800 kHz → 1.25 µs per bit
constexpr double WS2811_AUTO_PIXEL_DATA_RATE      = 800000.0;                
constexpr double WS2811_AUTO_PIXEL_NS_BIT_TOTAL   = (1.0 / WS2811_AUTO_PIXEL_DATA_RATE) * 1e9; 

constexpr double WS2811_AUTO_PIXEL_NS_BIT_0_HIGH  = 300.0;   // 0.3 µs
constexpr double WS2811_AUTO_PIXEL_NS_BIT_1_HIGH  = 900.0;   // 0.9 µs
constexpr double WS2811_AUTO_PIXEL_IDLE_TIME_US   = 300.0;   
constexpr double WS2811_AUTO_PIXEL_IDLE_TIME_NS   = WS2811_AUTO_PIXEL_IDLE_TIME_US * 1000.0;

// ----------------------------------------------------------------------------
// RMT clock → tick length
// ----------------------------------------------------------------------------
#ifndef RMT_BASE_CLK
#define RMT_BASE_CLK 80000000UL
#endif
#ifndef RMT_CLK_DIV
#define RMT_CLK_DIV 1
#endif

constexpr double RMT_TickLengthNS_AUTO = (1e9 / (RMT_BASE_CLK / RMT_CLK_DIV));  // e.g. 12.5 ns per tick

// ----------------------------------------------------------------------------
// Convert WS2811 timing to RMT ticks
// ----------------------------------------------------------------------------
constexpr uint16_t WS2811_AUTO_TICKS_BIT_0_HIGH = static_cast<uint16_t>(WS2811_AUTO_PIXEL_NS_BIT_0_HIGH / RMT_TickLengthNS_AUTO + 0.5);
constexpr uint16_t WS2811_AUTO_TICKS_BIT_0_LOW  = static_cast<uint16_t>((WS2811_AUTO_PIXEL_NS_BIT_TOTAL - WS2811_AUTO_PIXEL_NS_BIT_0_HIGH) / RMT_TickLengthNS_AUTO + 0.5);
constexpr uint16_t WS2811_AUTO_TICKS_BIT_1_HIGH = static_cast<uint16_t>(WS2811_AUTO_PIXEL_NS_BIT_1_HIGH / RMT_TickLengthNS_AUTO + 0.5);
constexpr uint16_t WS2811_AUTO_TICKS_BIT_1_LOW  = static_cast<uint16_t>((WS2811_AUTO_PIXEL_NS_BIT_TOTAL - WS2811_AUTO_PIXEL_NS_BIT_1_HIGH) / RMT_TickLengthNS_AUTO + 0.5);
constexpr uint16_t WS2811_AUTO_TICKS_IDLE       = static_cast<uint16_t>(50000.0 / RMT_TickLengthNS_AUTO + 0.5);

/*****************************************************************************************
 *  Constructor / Destructor
 *****************************************************************************************/

c_OutputWS2811Rmt::c_OutputWS2811Rmt(
    c_OutputMgr::e_OutputChannelIds OutputChannelId,
    gpio_num_t outputGpio,
    uart_port_t uart,
    c_OutputMgr::e_OutputType outputType)
    : c_OutputWS2811(OutputChannelId, outputGpio, uart, outputType)
{
}

c_OutputWS2811Rmt::~c_OutputWS2811Rmt() = default;

/*****************************************************************************************
 *  SetConfig — read JSON and configure hardware
 *****************************************************************************************/

bool c_OutputWS2811Rmt::SetConfig(ArduinoJson::JsonObject &jsonConfig)
{
    if (jsonConfig.contains("Pin"))
    {
        DataPin = gpio_num_t(jsonConfig["Pin"].as<int>());
    }

    // Build RMT configuration struct
    c_OutputRmt::OutputRmtConfig_t config = {};
    config.DataPin        = DataPin;
    config.RmtChannelId   = rmt_channel_t(OutputChannelId);
    config.idle_level     = RMT_IDLE_LEVEL_LOW;
    config.TxMethod       = c_OutputRmt::OutputRmtConfig_t::TX_METHOD_t::NORMAL;
    config.IntensityDataWidth = 8;
    config.TxIntensityDataStartingMask = 0x80;
    config.NumFrameStartBits = 0;
    config.NumIdleBits = 1;

    // Fill timing values
    config.T0H = WS2811_AUTO_TICKS_BIT_0_HIGH;
    config.T0L = WS2811_AUTO_TICKS_BIT_0_LOW;
    config.T1H = WS2811_AUTO_TICKS_BIT_1_HIGH;
    config.T1L = WS2811_AUTO_TICKS_BIT_1_LOW;
    config.InterFrameGapTicks = WS2811_AUTO_TICKS_IDLE;
    config.ColorOrder = c_OutputWS2811::ColorOrder_t::Order_GRB;

    // Save & apply configuration to RMT
    Rmt.Begin(config, this);

    ESP_LOGI("WS2811Rmt", 
        "RMT configured: TickLen=%.2f ns, T0H=%u T0L=%u T1H=%u T1L=%u Idle=%u", 
        RMT_TickLengthNS_AUTO,
        WS2811_AUTO_TICKS_BIT_0_HIGH, WS2811_AUTO_TICKS_BIT_0_LOW,
        WS2811_AUTO_TICKS_BIT_1_HIGH, WS2811_AUTO_TICKS_BIT_1_LOW,
        WS2811_AUTO_TICKS_IDLE);

    return true;
}

/*****************************************************************************************
 *  Begin — initialize RMT channel and start
 *****************************************************************************************/

void c_OutputWS2811Rmt::Begin()
{
    c_OutputWS2811::Begin();

    // Minimal config if nothing loaded from JSON
    ArduinoJson::JsonDocument json;
    json["Pin"] = static_cast<int>(DataPin);
    SetConfig(json.as<ArduinoJson::JsonObject>());
}

/*****************************************************************************************
 *  Poll — main update loop
 *****************************************************************************************/

uint32_t c_OutputWS2811Rmt::Poll()
{
    if (!Rmt.DriverIsSendingIntensityData())
    {
        Rmt.Poll(); // your Rmt class should already have this
    }
    return 0;
}

/*****************************************************************************************
 *  Pause output
 *****************************************************************************************/

void c_OutputWS2811Rmt::PauseOutput(bool State)
{
    Rmt.PauseOutput(State);
}

/*****************************************************************************************
 *  GetStatus — report current driver state
 *****************************************************************************************/

void c_OutputWS2811Rmt::GetStatus(ArduinoJson::JsonObject &jsonStatus)
{
    jsonStatus["Type"] = "WS2811RMT";
    jsonStatus["GPIO"] = static_cast<int>(DataPin);
    jsonStatus["ClkDiv"] = RMT_CLK_DIV;
    jsonStatus["TickNS"] = RMT_TickLengthNS_AUTO;
    jsonStatus["0H"] = WS2811_AUTO_TICKS_BIT_0_HIGH;
    jsonStatus["0L"] = WS2811_AUTO_TICKS_BIT_0_LOW;
    jsonStatus["1H"] = WS2811_AUTO_TICKS_BIT_1_HIGH;
    jsonStatus["1L"] = WS2811_AUTO_TICKS_BIT_1_LOW;
    jsonStatus["Idle"] = WS2811_AUTO_TICKS_IDLE;
}

#endif // defined(SUPPORT_OutputType_WS2811) && defined(ARDUINO_ARCH_ESP32)
