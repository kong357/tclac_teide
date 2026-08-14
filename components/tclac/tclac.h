/**
* Create by Miguel Ángel López on 20/07/19
* and modify by xaxexa
* Refactoring & component making:
* Nightingale with soldering iron 15.03.2024
**/

#ifndef TCL_ESP_TCL_H
#define TCL_ESP_TCL_H

#include <string>

#include "esphome.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"

namespace esphome {
namespace tclac {

#define SET_TEMP_MASK   0b00001111

// Number of times to repeat a command frame per transmission (workaround
// for marginal 3.3V->5V TX lines without a level converter). Repeats are
// non-blocking and spaced out in time so that each is a distinct fair attempt,
// rather than a queue that the AC controller swallows as a single frame.
// With a hardware level converter, 1 repeat is sufficient.
#ifdef REPEAT_TX
    #define TX_REPEAT               3
#else
    #define TX_REPEAT               1
#endif


// Pause between command frame retransmissions
#define TX_REPEAT_SPACING_MS    200
// "Listen before talk": do not transmit while the bus is busy receiving.
// Line quiet duration required before transmitting (25ms ~= 24 byte-times at 9600 baud):
#define BUS_QUIET_MS            25
// Response window after sending a poll request: no command frames are sent
// within this window until the response arrives (response ~64ms, starts in ~150ms)
#define POLL_RESPONSE_WINDOW_MS 400
// Maximum number of transmission deferrals due to a busy bus (after which it transmits as-is)
#define TX_MAX_DEFERS           12

#define MODE_POS        7
// Bit 0b00100000 in the mode byte indicates the DISPLAY state of the AC.
// It must not be included when determining the mode (otherwise, with display turned off,
// any mode would be parsed as "unrecognized" and fall back to default = AUTO).
#define DISPLAY_BIT     0b00100000
#define MODE_MASK       0b00001111

#define MODE_AUTO       0b00000101
#define MODE_COOL       0b00000001
#define MODE_DRY        0b00000011
#define MODE_FAN_ONLY   0b00000010
#define MODE_HEAT       0b00000100

#define FAN_SPEED_POS   8
#define FAN_QUIET_POS   33

#define FAN_AUTO        0b10000000  //auto
#define FAN_QUIET       0x80        //silent
#define FAN_LOW         0b10010000  //  |
#define FAN_MIDDLE      0b11000000  //  ||
#define FAN_MEDIUM      0b10100000  //  |||
#define FAN_HIGH        0b11010000  //  ||||
#define FAN_FOCUS       0b10110000  //  |||||
#define FAN_DIFFUSE     0b10000000  //  POWER [7]
#define FAN_SPEED_MASK  0b11110000  //FAN SPEED MASK

#define SWING_POS           10
#define SWING_OFF           0b00000000
#define SWING_HORIZONTAL    0b00100000
#define SWING_VERTICAL      0b01000000
#define SWING_BOTH          0b01100000
#define SWING_MODE_MASK     0b01100000

using climate::ClimateCall;
using climate::ClimateMode;
using climate::ClimatePreset;
using climate::ClimateTraits;
using climate::ClimateFanMode;
using climate::ClimateSwingMode;

enum class VerticalSwingDirection : uint8_t {
    UP_DOWN = 0,
    UPSIDE = 1,
    DOWNSIDE = 2,
};
enum class HorizontalSwingDirection : uint8_t {
    LEFT_RIGHT = 0,
    LEFTSIDE = 1,
    CENTER = 2,
    RIGHTSIDE = 3,
};
enum class AirflowVerticalDirection : uint8_t {
    LAST = 0,
    MAX_UP = 1,
    UP = 2,
    CENTER = 3,
    DOWN = 4,
    MAX_DOWN = 5,
};
enum class AirflowHorizontalDirection : uint8_t {
    LAST = 0,
    MAX_LEFT = 1,
    LEFT = 2,
    CENTER = 3,
    RIGHT = 4,
    MAX_RIGHT = 5,
};

class tclacClimate : public climate::Climate, public esphome::uart::UARTDevice, public PollingComponent {

    private:
        uint8_t checksum;
        uint8_t check = 0;
        // Control dataTX frame consists of 38 bytes
        uint8_t dataTX[38];
        // dataRX can swell up to 68 bytes in certain AC models
        uint8_t dataRX[68];
        // Status polling command packet
        uint8_t poll[8] = {0xBB,0x00,0x01,0x04,0x02,0x01,0x00,0xBD};
        // Initialization and initial population of switch state variables
        bool beeper_status_;
        bool display_status_;
        bool force_mode_status_;
        uint8_t switch_preset = 0;
        bool module_display_status_;
        uint8_t switch_fan_mode = 0;
        bool is_call_control = false;
        uint8_t switch_swing_mode = 0;
        int target_temperature_set = 0;
        uint8_t switch_climate_mode = 0;
        bool allow_take_control = false;

        // "Listen before talk": line activity timestamps
        uint32_t last_rx_ms_ = 0;    // timestamp of last byte received from AC
        uint32_t poll_sent_ms_ = 0;  // timestamp when last status poll was sent
        uint8_t tx_size_ = 0;        // size of frame being sent (used for retries)

        bool bus_quiet_();
        void try_send_frame_(uint8_t attempt, uint8_t defers_left);
        
        esphome::climate::ClimateTraits traits_;
        
    public:

        tclacClimate() : PollingComponent(5 * 1000) {
            checksum = 0;
        }

        void readData();
        void takeControl();
        void loop() override;
        void setup() override;
        void update() override;
        void set_beeper_state(bool state);
        void set_display_state(bool disp_state);
        // Actual AC display state (synchronized from status frames in readData)
		
        bool get_display_state() { return this->display_status_; }
        void dataShow(bool flow, bool shine);
        void set_force_mode_state(bool f_state);
        void set_rx_led_pin(GPIOPin *rx_led_pin);
        void set_tx_led_pin(GPIOPin *tx_led_pin);
        void sendData(uint8_t * message, uint8_t size);
        void set_module_display_state(bool d_state);
        static std::string getHex(const uint8_t *message, size_t size);
        static uint8_t getChecksum(const uint8_t * message, size_t size);
        void set_vertical_airflow(AirflowVerticalDirection v_airflow);
        void set_horizontal_airflow(AirflowHorizontalDirection h_airflow);
        void set_vertical_swing_direction(VerticalSwingDirection vs_direction);
        void set_horizontal_swing_direction(HorizontalSwingDirection hs_direction);
        void set_supported_presets(climate::ClimatePresetMask presets);
        void set_supported_modes(climate::ClimateModeMask modes);
        void set_supported_fan_modes(climate::ClimateFanModeMask fan_modes);
        void set_supported_swing_modes(climate::ClimateSwingModeMask swing_modes);
        
    protected:
        GPIOPin *rx_led_pin_;
        GPIOPin *tx_led_pin_;
        ClimateTraits traits() override;
        climate::ClimateModeMask supported_modes_{};
        AirflowVerticalDirection vertical_direction_;
        climate::ClimatePresetMask supported_presets_{};
        AirflowHorizontalDirection horizontal_direction_;
        VerticalSwingDirection vertical_swing_direction_;
        climate::ClimateFanModeMask supported_fan_modes_{};
        HorizontalSwingDirection horizontal_swing_direction_;
        climate::ClimateSwingModeMask supported_swing_modes_{};
        void control(const climate::ClimateCall &call) override;
};
}
}

#endif //TCL_ESP_TCL_H