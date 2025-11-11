#include "output/OutputWS2811Rmt.hpp"
#include "output/OutputMgr.hpp"

#if defined(SUPPORT_OutputType_WS2811) && defined(ARDUINO_ARCH_ESP32)

/*****************************************************************************************
 * WS2811 Timing (berechnet für 80 MHz RMT Clock)
 *****************************************************************************************/

// WS2811 = 800 kHz → 1.25 µs pro Bit
constexpr double WS2811_PIXEL_DATA_RATE = 800000.0;
constexpr double WS2811_BIT_TOTAL_NS    = (1.0 / WS2811_PIXEL_DATA_RATE) * 1e9; // 1250 ns

// Datenblattwerte
constexpr double WS2811_BIT0_HIGH_NS = 300.0;
constexpr double WS2811_BIT1_HIGH_NS = 900.0;
constexpr double WS2811_RESET_TIME_NS = 50000.0; // 50 µs

// RMT Clock: 80 MHz (12.5 ns pro Tick)
#define RMT_BASE_CLK 80000000UL
#define RMT_CLK_DIV  1
constexpr double RMT_TICK_NS = (1e9 / (RMT_BASE_CLK / RMT_CLK_DIV));

// Tickwerte berechnen
constexpr uint16_t WS2811_T0H = static_cast<uint16_t>(WS2811_BIT0_HIGH_NS / RMT_TICK_NS + 0.5);
constexpr uint16_t WS2811_T0L = static_cast<uint16_t>((WS2811_BIT_TOTAL_NS - WS2811_BIT0_HIGH_NS) / RMT_TICK_NS + 0.5);
constexpr uint16_t WS2811_T1H = static_cast<uint16_t>(WS2811_BIT1_HIGH_NS / RMT_TICK_NS + 0.5);
constexpr uint16_t WS2811_T1L = static_cast<uint16_t>((WS2811_BIT_TOTAL_NS - WS2811_BIT1_HIGH_NS) / RMT_TICK_NS + 0.5);
constexpr uint16_t WS2811_TIDLE = static_cast<uint16_t>(WS2811_RESET_TIME_NS / RMT_TICK_NS + 0.5);

/*****************************************************************************************
 * Konstruktor / Destruktor
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
 * SetConfig — aus JSON lesen + RMT konfigurieren
 *****************************************************************************************/

bool c_OutputWS2811Rmt::SetConfig(ArduinoJson::JsonObject &jsonConfig)
{
    if (jsonConfig.containsKey("Pin"))
    {
        DataPin = gpio_num_t(jsonConfig["Pin"].as<int>());
    }

    c_OutputRmt::OutputRmtConfig_t cfg = {};
    cfg.DataPin = DataPin;
    cfg.RmtChannelId = rmt_channel_t(OutputChannelId);
    cfg.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg.NumFrameStartBits = 0;
    cfg.NumIdleBits = 1;
    cfg.IntensityDataWidth = 8;
    cfg.SendEndOfFrameBits = true;
    cfg.SendInterIntensityBits = false;
    cfg.RmtInterruptRate = c_OutputRmt::OutputRmtInterruptRate_t::RMT_INTERRUPT_RATE_EVERY_FRAME;
    cfg.pPixelDataSource = this;

    // Taktung für WS2811
    cfg.TicksZeroHigh = WS2811_T0H;
    cfg.TicksZeroLow  = WS2811_T0L;
    cfg.TicksOneHigh  = WS2811_T1H;
    cfg.TicksOneLow   = WS2811_T1L;
    cfg.InterFrameGapTicks = WS2811_TIDLE;

    // Startmaske (MSB first)
    cfg.TxIntensityDataStartingMask = 0x80;

    Rmt.Begin(cfg, this);

    ESP_LOGI("WS2811Rmt",
             "Configured WS2811 RMT: Tick=%.2fns 0H=%u 0L=%u 1H=%u 1L=%u Idle=%u",
             RMT_TICK_NS, WS2811_T0H, WS2811_T0L, WS2811_T1H, WS2811_T1L, WS2811_TIDLE);

    return true;
}

/*****************************************************************************************
 * Begin — Initialisierung
 *****************************************************************************************/

void c_OutputWS2811Rmt::Begin()
{
    c_OutputWS2811::Begin();

    // Minimale Default-Konfiguration
    ArduinoJson::DynamicJsonDocument json(256);
    json["Pin"] = static_cast<int>(DataPin);

    ArduinoJson::JsonObject obj = json.as<ArduinoJson::JsonObject>();
    SetConfig(obj);
}

/*****************************************************************************************
 * Poll — Hauptloop für WS2811-RMT
 *****************************************************************************************/

uint32_t c_OutputWS2811Rmt::Poll()
{
    Rmt.RmtPoll();  // dein RMT-Treiber führt das Sendesystem aus
    return 0;
}

/*****************************************************************************************
 * Status
 *****************************************************************************************/

void c_OutputWS2811Rmt::GetStatus(ArduinoJson::JsonObject &jsonStatus)
{
    jsonStatus["Type"] = "WS2811-RMT";
    jsonStatus["Pin"] = static_cast<int>(DataPin);
    jsonStatus["TickNS"] = RMT_TICK_NS;
    jsonStatus["T0H"] = WS2811_T0H;
    jsonStatus["T0L"] = WS2811_T0L;
    jsonStatus["T1H"] = WS2811_T1H;
    jsonStatus["T1L"] = WS2811_T1L;
    jsonStatus["Idle"] = WS2811_TIDLE;
}

void c_OutputWS2811Rmt::PauseOutput(bool State)
{
    Rmt.PauseOutput(State);
}

#endif // SUPPORT_OutputType_WS2811 && ARDUINO_ARCH_ESP32
