#include "output/OutputWS2811Rmt.hpp"
#include "output/OutputMgr.hpp"

#if defined(SUPPORT_OutputType_WS2811) && defined(ARDUINO_ARCH_ESP32)

/*****************************************************************************************
 * WS2811-Timings (berechnet für 80 MHz RMT-Clock)
 *****************************************************************************************/

constexpr double WS2811_DATA_RATE_HZ = 800000.0;                    // 800 kHz
constexpr double WS2811_NS_BIT_TOTAL = (1.0 / WS2811_DATA_RATE_HZ) * 1e9; // 1250 ns
constexpr double WS2811_NS_BIT0_HIGH = 300.0;
constexpr double WS2811_NS_BIT1_HIGH = 900.0;
constexpr double WS2811_NS_RESET     = 50000.0;                     // 50 µs

#define RMT_BASE_CLK 80000000UL
#define RMT_CLK_DIV  1
constexpr double RMT_TICK_NS = (1e9 / (RMT_BASE_CLK / RMT_CLK_DIV)); // 12.5 ns

constexpr uint16_t WS2811_T0H   = uint16_t(WS2811_NS_BIT0_HIGH / RMT_TICK_NS + 0.5);
constexpr uint16_t WS2811_T0L   = uint16_t((WS2811_NS_BIT_TOTAL - WS2811_NS_BIT0_HIGH) / RMT_TICK_NS + 0.5);
constexpr uint16_t WS2811_T1H   = uint16_t(WS2811_NS_BIT1_HIGH / RMT_TICK_NS + 0.5);
constexpr uint16_t WS2811_T1L   = uint16_t((WS2811_NS_BIT_TOTAL - WS2811_NS_BIT1_HIGH) / RMT_TICK_NS + 0.5);
constexpr uint16_t WS2811_IFG   = uint16_t(WS2811_NS_RESET / RMT_TICK_NS + 0.5);

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
    cfg.RmtChannelId    = rmt_channel_t(OutputChannelId);
    cfg.DataPin         = DataPin;
    cfg.idle_level      = RMT_IDLE_LEVEL_LOW;
    cfg.NumFrameStartBits = 0;
    cfg.NumIdleBits       = 1;
    cfg.IntensityDataWidth = 8;
    cfg.SendEndOfFrameBits = true;
    cfg.SendInterIntensityBits = false;
    cfg.pPixelDataSource = this;

    // Timings → entsprechend deiner OutputRmtConfig_t
    cfg.T0H = WS2811_T0H;
    cfg.T0L = WS2811_T0L;
    cfg.T1H = WS2811_T1H;
    cfg.T1L = WS2811_T1L;
    cfg.ifgTicks = WS2811_IFG;

    cfg.TxIntensityDataStartingMask = 0x80;

    // RMT Start
    Rmt.Begin(cfg, this);

    ESP_LOGI("WS2811Rmt",
             "WS2811 RMT konfiguriert GPIO=%d T0H=%u T0L=%u T1H=%u T1L=%u IFG=%u (%.2f ns per tick)",
             DataPin, WS2811_T0H, WS2811_T0L, WS2811_T1H, WS2811_T1L, WS2811_IFG, RMT_TICK_NS);

    return true;
}

/*****************************************************************************************
 * Begin — Initialisierung
 *****************************************************************************************/

void c_OutputWS2811Rmt::Begin()
{
    c_OutputWS2811::Begin();

    // Default-Konfig erzeugen → aktuelle Pin-Nummer übernehmen
    ArduinoJson::DynamicJsonDocument json(256);
    json["Pin"] = static_cast<int>(DataPin);
    auto obj = json.as<ArduinoJson::JsonObject>();

    SetConfig(obj);
}

/*****************************************************************************************
 * Poll — Hauptloop für RMT
 *****************************************************************************************/

uint32_t c_OutputWS2811Rmt::Poll()
{
    Rmt.RmtPoll();  // Aufruf aus deinem RMT-Treiber
    return 0;
}

/*****************************************************************************************
 * Status
 *****************************************************************************************/

void c_OutputWS2811Rmt::GetStatus(ArduinoJson::JsonObject &jsonStatus)
{
    jsonStatus["Type"]   = "WS2811-RMT";
    jsonStatus["Pin"]    = static_cast<int>(DataPin);
    jsonStatus["T0H"]    = WS2811_T0H;
    jsonStatus["T0L"]    = WS2811_T0L;
    jsonStatus["T1H"]    = WS2811_T1H;
    jsonStatus["T1L"]    = WS2811_T1L;
    jsonStatus["IFG"]    = WS2811_IFG;
    jsonStatus["TickNS"] = RMT_TICK_NS;
}

void c_OutputWS2811Rmt::PauseOutput(bool State)
{
    Rmt.PauseOutput(State);
}

#endif // SUPPORT_OutputType_WS2811 && ARDUINO_ARCH_ESP32
