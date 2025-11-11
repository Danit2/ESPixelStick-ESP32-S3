#include "output/OutputWS2811Rmt.hpp"
#include "output/OutputMgr.hpp"

#if defined(SUPPORT_OutputType_WS2811) && defined(ARDUINO_ARCH_ESP32)

/*****************************************************************************************
 * WS2811 Timing definitions (auto-calculated for ESP32-S3 RMT)
 *****************************************************************************************/

// WS2811 runs at 800 kHz → 1.25 µs per bit
constexpr double WS2811_AUTO_PIXEL_DATA_RATE      = 800000.0;                // 800 kHz
constexpr double WS2811_AUTO_PIXEL_NS_BIT_TOTAL   = (1.0 / WS2811_AUTO_PIXEL_DATA_RATE) * 1e9; // ns per bit

constexpr double WS2811_AUTO_PIXEL_NS_BIT_0_HIGH  = 300.0;   // 0.3 µs high for logic 0
constexpr double WS2811_AUTO_PIXEL_NS_BIT_1_HIGH  = 900.0;   // 0.9 µs high for logic 1
constexpr double WS2811_AUTO_PIXEL_IDLE_TIME_US   = 300.0;   // ≥50 µs required
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

c_OutputWS2811Rmt::~c_OutputWS2811Rmt()
{
}

/*****************************************************************************************
 *  SetConfig — read JSON and configure hardware
 *****************************************************************************************/

bool c_OutputWS2811Rmt::SetConfig(ArduinoJson::JsonObject &jsonConfig)
{
    // Optional: read JSON fields (if present)
    if (jsonConfig.containsKey("Pin"))
    {
        DataPin = gpio_num_t(jsonConfig["Pin"].as<int>());
    }

    // pass config to underlying RMT driver
    ArduinoJson::DynamicJsonDocument rmtDoc(1024);
    JsonObject rmtCfg = rmtDoc.to<JsonObject>();

    rmtCfg["DataPin"]    = static_cast<int>(DataPin);
    rmtCfg["ClkDiv"]     = RMT_CLK_DIV;
    rmtCfg["T0H"]        = WS2811_AUTO_TICKS_BIT_0_HIGH;
    rmtCfg["T0L"]        = WS2811_AUTO_TICKS_BIT_0_LOW;
    rmtCfg["T1H"]        = WS2811_AUTO_TICKS_BIT_1_HIGH;
    rmtCfg["T1L"]        = WS2811_AUTO_TICKS_BIT_1_LOW;
    rmtCfg["IdleTicks"]  = WS2811_AUTO_TICKS_IDLE;
    rmtCfg["ColorOrder"] = "GRB";

    bool ok = Rmt.SetConfig(rmtCfg);
    return ok;
}

/*****************************************************************************************
 *  Begin — initialize RMT channel and start
 *****************************************************************************************/

void c_OutputWS2811Rmt::Begin()
{
    c_OutputWS2811::Begin();

    // prepare minimal configuration for WS2811
    ArduinoJson::DynamicJsonDocument cfg(1024);
    JsonObject json = cfg.to<JsonObject>();
    json["Pin"] = static_cast<int>(DataPin);

    // Apply configuration and start RMT driver
    SetConfig(json);
    Rmt.Begin();

    ESP_LOGI("WS2811Rmt", "WS2811 RMT initialized on GPIO %d, clk_div=%d", DataPin, RMT_CLK_DIV);
}

/*****************************************************************************************
 *  Poll — main update loop
 *****************************************************************************************/

uint32_t c_OutputWS2811Rmt::Poll()
{
    // Delegate the actual sending to RMT object
    if (Rmt.DriverIsSendingIntensityData())
    {
        return 0;
    }

    // Tell RMT to send current pixel data
    Rmt.RmtPoll();
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
