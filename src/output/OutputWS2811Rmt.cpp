#include "output/OutputWS2811Rmt.hpp"
#include "output/OutputMgr.hpp"

#ifdef SUPPORT_OutputType_WS2811

/*****************************************************************************************
 * Timing definitions for WS2811 — auto-calculated for RMT base clock (ESP32-S3 compatible)
 *****************************************************************************************/

// WS2811 runs at 800 kHz → 1.25 µs per bit
constexpr double WS2811_AUTO_PIXEL_DATA_RATE      = 800000.0;                // 800 kHz
constexpr double WS2811_AUTO_PIXEL_NS_BIT_TOTAL   = (1.0 / WS2811_AUTO_PIXEL_DATA_RATE) * 1e9; // ns per bit

constexpr double WS2811_AUTO_PIXEL_NS_BIT_0_HIGH  = 300.0;
constexpr double WS2811_AUTO_PIXEL_NS_BIT_1_HIGH  = 900.0;
constexpr double WS2811_AUTO_PIXEL_IDLE_TIME_US   = 300.0;
constexpr double WS2811_AUTO_PIXEL_IDLE_TIME_NS   = WS2811_AUTO_PIXEL_IDLE_TIME_US * 1000.0;

#ifndef RMT_BASE_CLK
#define RMT_BASE_CLK 80000000UL
#endif

#ifndef RMT_CLK_DIV
#define RMT_CLK_DIV 1
#endif

constexpr double RMT_TickLengthNS_AUTO = (1e9 / (RMT_BASE_CLK / RMT_CLK_DIV));  // e.g. 12.5 ns per tick

constexpr uint16_t WS2811_AUTO_TICKS_BIT_0_HIGH = static_cast<uint16_t>(WS2811_AUTO_PIXEL_NS_BIT_0_HIGH / RMT_TickLengthNS_AUTO + 0.5);
constexpr uint16_t WS2811_AUTO_TICKS_BIT_0_LOW  = static_cast<uint16_t>((WS2811_AUTO_PIXEL_NS_BIT_TOTAL - WS2811_AUTO_PIXEL_NS_BIT_0_HIGH) / RMT_TickLengthNS_AUTO + 0.5);
constexpr uint16_t WS2811_AUTO_TICKS_BIT_1_HIGH = static_cast<uint16_t>(WS2811_AUTO_PIXEL_NS_BIT_1_HIGH / RMT_TickLengthNS_AUTO + 0.5);
constexpr uint16_t WS2811_AUTO_TICKS_BIT_1_LOW  = static_cast<uint16_t>((WS2811_AUTO_PIXEL_NS_BIT_TOTAL - WS2811_AUTO_PIXEL_NS_BIT_1_HIGH) / RMT_TickLengthNS_AUTO + 0.5);
constexpr uint16_t WS2811_AUTO_TICKS_IDLE       = static_cast<uint16_t>(50000.0 / RMT_TickLengthNS_AUTO + 0.5);

/*****************************************************************************************
 *  Constructor / Setup
 *****************************************************************************************/

c_OutputWS2811Rmt::c_OutputWS2811Rmt(c_OutputMgr::e_OutputChannelIds OutputChannelId,
                                     gpio_num_t outputGpio,
                                     uart_port_t uart,
                                     c_OutputMgr::e_OutputType outputType)
    : c_OutputWS2811(OutputChannelId, outputGpio, uart, outputType)
{
}

/*****************************************************************************************
 *  Internal hardware configuration for RMT
 *****************************************************************************************/

void c_OutputWS2811Rmt::ConfigureRmtHardware()
{
    OutputRmtConfig.RmtChannelId      = rmt_channel_t(OutputChannelId);
    OutputRmtConfig.DataPin           = gpio_num_t(DataPin);
    OutputRmtConfig.idle_level        = rmt_idle_level_t::RMT_IDLE_LEVEL_LOW;
    OutputRmtConfig.pPixelDataSource  = this;
    OutputRmtConfig.NumFrameStartBits = 0;
    OutputRmtConfig.NumIdleBits       = 1;
    OutputRmtConfig.CitrdsArray       = ConvertIntensityToRmtDataStream;

    OutputRmtConfig.ColorOrder        = c_OutputWS2811::ColorOrder_t::Order_GRB;
    OutputRmtConfig.IntensityDataWidth = 8;
    OutputRmtConfig.TxIntensityDataStartingMask = 0x80;

    Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID].duration0 = WS2811_AUTO_TICKS_BIT_0_HIGH;
    Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID].duration1 = WS2811_AUTO_TICKS_BIT_0_LOW;
    Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID].duration0  = WS2811_AUTO_TICKS_BIT_1_HIGH;
    Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID].duration1  = WS2811_AUTO_TICKS_BIT_1_LOW;
    Intensity2Rmt[RmtDataBitIdType_t::RMT_END_OF_FRAME].duration0     = WS2811_AUTO_TICKS_IDLE;
    Intensity2Rmt[RmtDataBitIdType_t::RMT_END_OF_FRAME].duration1     = 0;

    ESP_LOGI("WS2811", "RMT_TickLength=%.2f ns, 0H=%u 0L=%u 1H=%u 1L=%u Idle=%u",
             RMT_TickLengthNS_AUTO,
             WS2811_AUTO_TICKS_BIT_0_HIGH,
             WS2811_AUTO_TICKS_BIT_0_LOW,
             WS2811_AUTO_TICKS_BIT_1_HIGH,
             WS2811_AUTO_TICKS_BIT_1_LOW,
             WS2811_AUTO_TICKS_IDLE);
}

/*****************************************************************************************
 *  Begin — initialize RMT channel and start
 *****************************************************************************************/

void c_OutputWS2811Rmt::Begin()
{
    c_OutputWS2811::Begin();
    ConfigureRmtHardware();

    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = OutputRmtConfig.RmtChannelId;
    config.gpio_num = OutputRmtConfig.DataPin;
    config.mem_block_num = 1;
    config.tx_config.idle_level = OutputRmtConfig.idle_level;
    config.tx_config.idle_output_en = true;
    config.clk_div = RMT_CLK_DIV;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    ESP_LOGI("WS2811", "RMT initialized on GPIO %d (clk_div=%d)", OutputRmtConfig.DataPin, RMT_CLK_DIV);
}

#endif // SUPPORT_OutputType_WS2811
