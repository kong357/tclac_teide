/**
* Create by Miguel Ángel López on 20/07/19
* and modify by xaxexa
* Refactoring & component making:
* Nightingale with soldering iron 15.03.2024
**/
#include <cmath>

#include "esphome.h"
#include "esphome/core/defines.h"
#include "tclac.h"

namespace esphome{
namespace tclac{


ClimateTraits tclacClimate::traits() {
    auto traits = climate::ClimateTraits();
    traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
    
    // Responsibility disclosure: I took all of this from christoph5180
    if (this->supported_modes_.empty()) {
        traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
        traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);
    } else {
        for (auto mode : this->supported_modes_)
            traits.add_supported_mode(mode);
    }
    if (this->supported_presets_.empty()) {
        traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_NONE);
    } else {
        for (auto preset : this->supported_presets_)
            traits.add_supported_preset(preset);
    }
    if (this->supported_fan_modes_.empty()) {
        traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
    } else {
        for (auto fan_mode : this->supported_fan_modes_)
            traits.add_supported_fan_mode(fan_mode);
    }
    if (this->supported_swing_modes_.empty()) {
        traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
    } else {
        for (auto swing_mode : this->supported_swing_modes_)
            traits.add_supported_swing_mode(swing_mode);
    }

    return traits;
}


void tclacClimate::setup() {

#ifdef CONF_RX_LED
    this->rx_led_pin_->setup();
    this->rx_led_pin_->digital_write(false);
#endif
#ifdef CONF_TX_LED
    this->tx_led_pin_->setup();
    this->tx_led_pin_->digital_write(false);
#endif
}

void tclacClimate::loop()  {
    // If there is data in the UART buffer, read it
    if (esphome::uart::UARTDevice::available() > 0) {
        // Line is busy receiving — hold back sending command frames
        this->last_rx_ms_ = millis();
        dataShow(0, true);
        dataRX[0] = esphome::uart::UARTDevice::read();
        // If the received byte is not the header (0xBB), exit loop
        if (dataRX[0] != 0xBB) {
            ESP_LOGD("TCL", "Wrong byte");
            dataShow(0,0);
            return;
        }
        // If the header matches (0xBB), start reading the next 4 bytes sequentially
        // Sometimes, for certain AC units, a delay(5) is needed between packets. Who knows why, but it is necessary. Not always, but sometimes yes.
        // delay(5);
        dataRX[1] = esphome::uart::UARTDevice::read();
        // delay(5);
        dataRX[2] = esphome::uart::UARTDevice::read();
        // delay(5);
        dataRX[3] = esphome::uart::UARTDevice::read();
        // delay(5);
        dataRX[4] = esphome::uart::UARTDevice::read();

        //auto raw = getHex(dataRX, 5);
        //ESP_LOGD("TCL", "first 5 byte : %s ", raw.c_str());

        // OVERFLOW PROTECTION: 5th byte is payload length, then
        // dataRX[4]+1 bytes are read into dataRX+5. Only three frame lengths
        // are legitimate (0x37=61, 0x3b=65, 0x3e=68 bytes total). On a noisy line
        // read() might return -1 (0xFF) or garbage — which would make dataRX[4]+6
        // exceed dataRX[68] bounds. Drop such frame.
        if (dataRX[4] != 0x37 && dataRX[4] != 0x3b && dataRX[4] != 0x3e) {
            ESP_LOGW("TCL", "Bad frame length 0x%02X, dropped", dataRX[4]);
            while (esphome::uart::UARTDevice::available() > 0) esphome::uart::UARTDevice::read();
            dataShow(0,0);
            return;
        }

        // From the first 5 bytes we need the 5th — it contains message length.
        // read_array returns false on timeout (frame broken mid-way) —
        // then the buffer contains garbage from previous frame and cannot be parsed.
        if (!esphome::uart::UARTDevice::read_array(dataRX+5, dataRX[4]+1)) {
            ESP_LOGW("TCL", "Frame read timeout, dropped");
            dataShow(0,0);
            return;
        }

        // Calculate checksum:
        if (dataRX[4] == 0x3e){
            // For data packet length of 68 bytes
            check = getChecksum(dataRX, 68);
        } else if (dataRX[4] == 0x37){
            // For data packet length of 61 bytes
            check = getChecksum(dataRX, 61);
        } else {
            // For data packet length of 65 bytes
            check = getChecksum(dataRX, 65);
        }

        //raw = getHex(dataRX, sizeof(dataRX));
        //ESP_LOGD("TCL", "RX full : %s ", raw.c_str());
        
        // Verify checksum:
        if (dataRX[4] == 0x3e){
            // For data packet length of 68 bytes
            if (check != dataRX[67]) {
                ESP_LOGD("TCL", "Invalid checksum %x", check);
                this->dataShow(0,0);
                return;
            } else {
                //ESP_LOGD("TCL", "checksum OK %x", check);
            }
        } else if (dataRX[4] == 0x37){
            if (check != dataRX[60]) {
                // For data packet length of 61 bytes
                ESP_LOGD("TCL", "Invalid checksum %x", check);
                this->dataShow(0,0);
                return;
            } else {
                //ESP_LOGD("TCL", "checksum OK %x", check);
            }
        } else {
            if (check != dataRX[64]) {
                // For data packet length of 65 bytes
                ESP_LOGD("TCL", "Invalid checksum %x", check);
                this->dataShow(0,0);
                return;
            } else {
                //ESP_LOGD("TCL", "checksum OK %x", check);
            }
        }
        this->dataShow(0,0);
        // After reading everything from buffer, proceed with data parsing
        this->readData();
    }
}

void tclacClimate::update() {
    tclacClimate::dataShow(1,1);
    this->esphome::uart::UARTDevice::write_array(poll, sizeof(poll));
    // After polling, AC unit will start responding — command frames will wait
    this->poll_sent_ms_ = millis();
    //auto raw = tclacClimate::getHex(poll, sizeof(poll));
    ESP_LOGD("TCL", "Check status sended");
    tclacClimate::dataShow(1,0);
}

void tclacClimate::readData() {
    
    // This construction was proposed by Claude AI, I don't really understand such quirks, so leaving it as is.
    current_temperature = ((float)((dataRX[17] << 8) | dataRX[18]) / 374.0f - 32.0f) / 1.8f;
    
    target_temperature = (dataRX[FAN_SPEED_POS] & SET_TEMP_MASK) + 16;

    //ESP_LOGD("TCL", "TEMP: %f ", current_temperature);

    if (dataRX[MODE_POS] & ( 1 << 4)) {
        // If AC is ON, parse data for display
        ESP_LOGD("TCL", "AC is on");

        // Synchronize display state with actual (DISPLAY_BIT bit in mode byte):
        // Remote control switches display bypassing module, and without synchronization
        // the next module command would overwrite display state with old value. Only when AC is ON.

        this->display_status_ = (dataRX[MODE_POS] & DISPLAY_BIT) != 0;

        uint8_t modeswitch = MODE_MASK & dataRX[MODE_POS];
        uint8_t fanspeedswitch = FAN_SPEED_MASK & dataRX[FAN_SPEED_POS];
        uint8_t swingmodeswitch = SWING_MODE_MASK & dataRX[SWING_POS];

        switch (modeswitch) {
            case MODE_AUTO:
                this->mode = climate::CLIMATE_MODE_HEAT_COOL;
                break;
            case MODE_COOL:
                this->mode = climate::CLIMATE_MODE_COOL;
                break;
            case MODE_DRY:
                this->mode = climate::CLIMATE_MODE_DRY;
                break;
            case MODE_FAN_ONLY:
                this->mode = climate::CLIMATE_MODE_FAN_ONLY;
                break;
            case MODE_HEAT:
                this->mode = climate::CLIMATE_MODE_HEAT;
                break;
            default:
                this->mode = climate::CLIMATE_MODE_HEAT_COOL;
        }

        if ( dataRX[FAN_QUIET_POS] & FAN_QUIET) {
            fan_mode = climate::CLIMATE_FAN_QUIET;
        } else if (dataRX[MODE_POS] & FAN_DIFFUSE){
            fan_mode = climate::CLIMATE_FAN_DIFFUSE;
        } else {
            switch (fanspeedswitch) {
                case FAN_AUTO:
                    fan_mode = climate::CLIMATE_FAN_AUTO;
                    break;
                case FAN_LOW:
                    fan_mode = climate::CLIMATE_FAN_LOW;
                    break;
                case FAN_MIDDLE:
                    fan_mode = climate::CLIMATE_FAN_MIDDLE;
                    break;
                case FAN_MEDIUM:
                    fan_mode = climate::CLIMATE_FAN_MEDIUM;
                    break;
                case FAN_HIGH:
                    fan_mode = climate::CLIMATE_FAN_HIGH;
                    break;
                case FAN_FOCUS:
                    fan_mode = climate::CLIMATE_FAN_FOCUS;
                    break;
                default:
                    fan_mode = climate::CLIMATE_FAN_AUTO;
            }
        }

        switch (swingmodeswitch) {
            case SWING_OFF: 
                swing_mode = climate::CLIMATE_SWING_OFF;
                break;
            case SWING_HORIZONTAL:
                swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
                break;
            case SWING_VERTICAL:
                swing_mode = climate::CLIMATE_SWING_VERTICAL;
                break;
            case SWING_BOTH:
                swing_mode = climate::CLIMATE_SWING_BOTH;
                break;
        }
        
        // Preset data processing
        preset = ClimatePreset::CLIMATE_PRESET_NONE;
        if (dataRX[7] & (1 << 6)){
            preset = ClimatePreset::CLIMATE_PRESET_ECO;
        } else if (dataRX[9] & (1 << 2)){
            preset = ClimatePreset::CLIMATE_PRESET_COMFORT;
        } else if (dataRX[19] & (1 << 0)){
            preset = ClimatePreset::CLIMATE_PRESET_SLEEP;
        }
        
    } else {
        ESP_LOGD("TCL", "AC is OFF");
        // If AC is OFF, all modes show as OFF
        this->mode = climate::CLIMATE_MODE_OFF;
        //fan_mode = climate::CLIMATE_FAN_OFF;
        this->swing_mode = climate::CLIMATE_SWING_OFF;
        this->preset = ClimatePreset::CLIMATE_PRESET_NONE;
    }
    // Publish state
    this->publish_state();
    allow_take_control = true;
   }

// Climate control
void tclacClimate::control(const climate::ClimateCall &call) {
    
    ESP_LOGD("TCL", "Call from UI");

    // ECHO-LOOP PROTECTION: command that doesn't change current state is ignored.
    // Integrations (e.g., Yandex Alice client or MQTT bridge) might reflect
    // state back into command; without this check an echo frame would re-trigger
    // control() -> state publish -> echo again, creating a storm.
    bool changed = false;
    if (call.get_mode().has_value() && *call.get_mode() != this->mode) changed = true;
    if (call.get_target_temperature().has_value()
        && (int) *call.get_target_temperature() != (int) this->target_temperature) changed = true;
    if (call.get_fan_mode().has_value()
        && (!this->fan_mode.has_value() || *call.get_fan_mode() != this->fan_mode.value())) changed = true;
    if (call.get_swing_mode().has_value() && *call.get_swing_mode() != this->swing_mode) changed = true;
    if (call.get_preset().has_value()
        && (!this->preset.has_value() || *call.get_preset() != this->preset.value())) changed = true;
    if (!changed) {
        ESP_LOGD("TCL", "Call from UI: no changes, skipped");
        return;
    }

    // And this below I borrowed from Vi3jo.

    if (call.get_mode().has_value()) this->mode = *call.get_mode();
    if (call.get_target_temperature().has_value()) this->target_temperature = *call.get_target_temperature();
    if (call.get_fan_mode().has_value()) this->fan_mode = *call.get_fan_mode();
    if (call.get_swing_mode().has_value()) this->swing_mode = *call.get_swing_mode();
    if (call.get_preset().has_value()) this->preset = *call.get_preset();
    
    this->publish_state();
    this->takeControl();
    this->allow_take_control = true;
}
    
    
void tclacClimate::takeControl() {
    
    dataTX[7]  = 0b00000000;
    dataTX[8]  = 0b00000000;
    dataTX[9]  = 0b00000000;
    dataTX[10] = 0b00000000;
    dataTX[11] = 0b00000000;
    dataTX[19] = 0b00000000;
    dataTX[32] = 0b00000000;
    dataTX[33] = 0b00000000;
    
    // Protection against garbage in setpoint byte: until first status frame
    // target_temperature = NaN, and (int)NaN is undefined behavior.
    if (std::isnan(target_temperature) || target_temperature < 16 || target_temperature > 31) {
        target_temperature = 24;
    }
    uint8_t target_temperature_set = 31-(int)target_temperature;
    
    // Enable or disable beeper depending on settings switch
    if (beeper_status_){
        ESP_LOGD("TCL", "Beep mode ON");
        dataTX[7] += 0b00100000;
    } else {
        ESP_LOGD("TCL", "Beep mode OFF");
        dataTX[7] += 0b00000000;
    }
    
    // Enable or disable display on AC depending on settings switch
    // Enable display only if AC is in an active operating mode
    
    // ATTENTION! When display is turned off, AC unit forcibly switches to auto mode!
    
    if ((display_status_) && (mode != climate::CLIMATE_MODE_OFF)){
        ESP_LOGD("TCL", "Dispaly turn ON");
        dataTX[7] += 0b01000000;
    } else {
        ESP_LOGD("TCL", "Dispaly turn OFF");
        dataTX[7] += 0b00000000;
    }
        
    // Configure AC operating mode
    switch (this->mode) {
        case climate::CLIMATE_MODE_OFF:
            dataTX[7] += 0b00000000;
            dataTX[8] += 0b00000000;
            break;
        case climate::CLIMATE_MODE_HEAT_COOL:
            dataTX[7] += 0b00000100;
            dataTX[8] += 0b00001000;
            break;
        case climate::CLIMATE_MODE_COOL:
            dataTX[7] += 0b00000100;
            dataTX[8] += 0b00000011;    
            break;
        case climate::CLIMATE_MODE_DRY:
            dataTX[7] += 0b00000100;
            dataTX[8] += 0b00000010;    
            break;
        case climate::CLIMATE_MODE_FAN_ONLY:
            dataTX[7] += 0b00000100;
            dataTX[8] += 0b00000111;    
            break;
        case climate::CLIMATE_MODE_HEAT:
            dataTX[7] += 0b00000100;
            dataTX[8] += 0b00000001;    
            break;
        case climate::CLIMATE_MODE_AUTO:
            // This ESPHome mode is not used by TCLAC component.
            // For automatic mode, TCLAC uses CLIMATE_MODE_HEAT_COOL.
            break;
    }

    // Configure fan speed
    if (this->fan_mode.has_value()) {
        switch(*this->fan_mode) {
            case climate::CLIMATE_FAN_AUTO:
                dataTX[8]   += 0b00000000;
                dataTX[10]  += 0b00000000;
                break;
            case climate::CLIMATE_FAN_QUIET:
                dataTX[8]   += 0b10000000;
                dataTX[10]  += 0b00000000;
                break;
            case climate::CLIMATE_FAN_LOW:
                dataTX[8]   += 0b00000000;
                dataTX[10]  += 0b00000001;
                break;
            case climate::CLIMATE_FAN_MIDDLE:
                dataTX[8]   += 0b00000000;
                dataTX[10]  += 0b00000110;
                break;
            case climate::CLIMATE_FAN_MEDIUM:
                dataTX[8]   += 0b00000000;
                dataTX[10]  += 0b00000011;
                break;
            case climate::CLIMATE_FAN_HIGH:
                dataTX[8]   += 0b00000000;
                dataTX[10]  += 0b00000111;
                break;
            case climate::CLIMATE_FAN_FOCUS:
                dataTX[8]   += 0b00000000;
                dataTX[10]  += 0b00000101;
                break;
            case climate::CLIMATE_FAN_DIFFUSE:
                dataTX[8]   += 0b01000000;
                dataTX[10]  += 0b00000000;
                break;
            case climate::CLIMATE_FAN_ON:
            case climate::CLIMATE_FAN_OFF:
                // These generic ESPHome modes are not supported by TCLAC protocol.
                break;
        }
    }
    
    // Set louver swing mode
    switch(this->swing_mode) {
        case climate::CLIMATE_SWING_OFF:
            dataTX[10]  += 0b00000000;
            dataTX[11]  += 0b00000000;
            break;
        case climate::CLIMATE_SWING_VERTICAL:
            dataTX[10]  += 0b00111000;
            dataTX[11]  += 0b00000000;
            break;
        case climate::CLIMATE_SWING_HORIZONTAL:
            dataTX[10]  += 0b00000000;
            dataTX[11]  += 0b00001000;
            break;
        case climate::CLIMATE_SWING_BOTH:
            dataTX[10]  += 0b00111000;
            dataTX[11]  += 0b00001000;  
            break;
    }
    
    // Set AC presets
    if (this->preset.has_value()) {
        switch(*this->preset) {
            case ClimatePreset::CLIMATE_PRESET_NONE:
                break;
            case ClimatePreset::CLIMATE_PRESET_ECO:
                dataTX[7]   += 0b10000000;
                break;
            case ClimatePreset::CLIMATE_PRESET_SLEEP:
                dataTX[19]  += 0b00000001;
                break;
            case ClimatePreset::CLIMATE_PRESET_COMFORT:
                dataTX[8]   += 0b00010000;
                break;
            case ClimatePreset::CLIMATE_PRESET_HOME:
            case ClimatePreset::CLIMATE_PRESET_AWAY:
            case ClimatePreset::CLIMATE_PRESET_BOOST:
            case ClimatePreset::CLIMATE_PRESET_ACTIVITY:
                // These standard ESPHome presets are not supported by TCLAC protocol.
                break;
        }
    }

        // Louver mode
        //  Vertical louver
        //      Vertical louver swing [byte 10, mask 00111000]:
        //          000 - Swing disabled, louver in last position or fixed
        //          111 - Swing enabled in selected mode
        //      Vertical louver swing mode (fixed position mode doesn't matter if swing is enabled) [byte 32, mask 00011000]:
        //          01 - top to bottom swing, DEFAULT
        //          10 - top half swing
        //          11 - bottom half swing
        //      Louver fixed position mode (swing mode doesn't matter if swing is disabled) [byte 32, mask 00000111]:
        //          000 - no fix, DEFAULT
        //          001 - fix at top
        //          010 - fix between top and middle
        //          011 - fix at middle
        //          100 - fix between middle and bottom
        //          101 - fix at bottom
        //  Horizontal louvers
        //      Horizontal louvers swing [byte 11, mask 00001000]:
        //          0 - Swing disabled, louvers in last position or fixed
        //          1 - Swing enabled in selected mode
        //      Horizontal louvers swing mode (fixed position mode doesn't matter if swing is enabled) [byte 33, mask 00111000]:
        //          001 - left to right swing, DEFAULT
        //          010 - left side swing
        //          011 - center swing
        //          100 - right side swing
        //      Horizontal louvers fixed position mode (swing mode doesn't matter if swing is disabled) [byte 33, mask 00000111]:
        //          000 - no fix, DEFAULT
        //          001 - fix at left
        //          010 - fix between left side and center
        //          011 - fix at center
        //          100 - fix between center and right side
        //          101 - fix at right
        
        
    // Set vertical louver swing mode
    switch(vertical_swing_direction_) {
        case VerticalSwingDirection::UP_DOWN:
            dataTX[32]  += 0b00001000;
            ESP_LOGD("TCL", "Vertical swing: up-down");
            break;
        case VerticalSwingDirection::UPSIDE:
            dataTX[32]  += 0b00010000;
            ESP_LOGD("TCL", "Vertical swing: upper");
            break;
        case VerticalSwingDirection::DOWNSIDE:
            dataTX[32]  += 0b00011000;
            ESP_LOGD("TCL", "Vertical swing: downer");
            break;
    }
    // Set horizontal louvers swing mode
    switch(horizontal_swing_direction_) {
        case HorizontalSwingDirection::LEFT_RIGHT:
            dataTX[33]  += 0b00001000;
            ESP_LOGD("TCL", "Horizontal swing: left-right");
            break;
        case HorizontalSwingDirection::LEFTSIDE:
            dataTX[33]  += 0b00010000;
            ESP_LOGD("TCL", "Horizontal swing: lefter");
            break;
        case HorizontalSwingDirection::CENTER:
            dataTX[33]  += 0b00011000;
            ESP_LOGD("TCL", "Horizontal swing: center");
            break;
        case HorizontalSwingDirection::RIGHTSIDE:
            dataTX[33]  += 0b00100000;
            ESP_LOGD("TCL", "Horizontal swing: righter");
            break;
    }
    // Set vertical louver fixed position
    switch(vertical_direction_) {
        case AirflowVerticalDirection::LAST:
            dataTX[32]  += 0b00000000;
            ESP_LOGD("TCL", "Vertical fix: last position");
            break;
        case AirflowVerticalDirection::MAX_UP:
            dataTX[32]  += 0b00000001;
            ESP_LOGD("TCL", "Vertical fix: up");
            break;
        case AirflowVerticalDirection::UP:
            dataTX[32]  += 0b00000010;
            ESP_LOGD("TCL", "Vertical fix: upper");
            break;
        case AirflowVerticalDirection::CENTER:
            dataTX[32]  += 0b00000011;
            ESP_LOGD("TCL", "Vertical fix: center");
            break;
        case AirflowVerticalDirection::DOWN:
            dataTX[32]  += 0b00000100;
            ESP_LOGD("TCL", "Vertical fix: downer");
            break;
        case AirflowVerticalDirection::MAX_DOWN:
            dataTX[32]  += 0b00000101;
            ESP_LOGD("TCL", "Vertical fix: down");
            break;
    }
    // Set horizontal louvers fixed position
    switch(horizontal_direction_) {
        case AirflowHorizontalDirection::LAST:
            dataTX[33]  += 0b00000000;
            ESP_LOGD("TCL", "Horizontal fix: last position");
            break;
        case AirflowHorizontalDirection::MAX_LEFT:
            dataTX[33]  += 0b00000001;
            ESP_LOGD("TCL", "Horizontal fix: left");
            break;
        case AirflowHorizontalDirection::LEFT:
            dataTX[33]  += 0b00000010;
            ESP_LOGD("TCL", "Horizontal fix: lefter");
            break;
        case AirflowHorizontalDirection::CENTER:
            dataTX[33]  += 0b00000011;
            ESP_LOGD("TCL", "Horizontal fix: center");
            break;
        case AirflowHorizontalDirection::RIGHT:
            dataTX[33]  += 0b00000100;
            ESP_LOGD("TCL", "Horizontal fix: righter");
            break;
        case AirflowHorizontalDirection::MAX_RIGHT:
            dataTX[33]  += 0b00000101;
            ESP_LOGD("TCL", "Horizontal fix: right");
            break;
    }

    // Set temperature
    dataTX[9] = target_temperature_set;
        
    // Assemble byte array to send to AC
    dataTX[0] = 0xBB;   // Header start byte
    dataTX[1] = 0x00;   // Header start byte
    dataTX[2] = 0x01;   // Header start byte
    dataTX[3] = 0x03;   // 0x03 - control, 0x04 - poll
    dataTX[4] = 0x20;   // 0x20 - control, 0x19 - poll
    dataTX[5] = 0x03;   // ??
    dataTX[6] = 0x01;   // ??
    //dataTX[7] = 0x64; // eco,display,beep,ontimerenable, offitimerenable,power,0,0
    //dataTX[8] = 0x08; // mute,0,turbo,health, mode(4) mode 01 heat, 02 dry, 03 cool, 07 fan, 08 auto, health(+16), 41=turbo-heat 43=turbo-cool (turbo = 0x40+ 0x01..0x08)
    //dataTX[9] = 0x0f; // 0 -31 ;    15 - 16 0,0,0,0, temp(4) settemp 31 - x
    //dataTX[10] = 0x00;    // 0,timerindicator,swingv(3),fan(3) fan+swing modes //0=auto 1=low 2=med 3=high
    //dataTX[11] = 0x00;    // 0,offtimer(6),0
    dataTX[12] = 0x00;  // fahrenheit,ontimer(6),0 cf 80=f 0=c
    dataTX[13] = 0x01;  // ??
    dataTX[14] = 0x00;  // 0,0,halfdegree,0,0,0,0,0
    dataTX[15] = 0x00;  // ??
    dataTX[16] = 0x00;  // ??
    dataTX[17] = 0x00;  // ??
    dataTX[18] = 0x00;  // ??
    //dataTX[19] = 0x00;    // sleep on = 1 off=0
    dataTX[20] = 0x00;  // ??
    dataTX[21] = 0x00;  // ??
    dataTX[22] = 0x00;  // ??
    dataTX[23] = 0x00;  // ??
    dataTX[24] = 0x00;  // ??
    dataTX[25] = 0x00;  // ??
    dataTX[26] = 0x00;  // ??
    dataTX[27] = 0x00;  // ??
    dataTX[28] = 0x00;  // ??
    dataTX[29] = 0x00;  // ??
    dataTX[30] = 0x00;  // ??
    dataTX[31] = 0x00;  // ??
    //dataTX[32] = 0x00;    // 0,0,0,vertical swing mode(2),vertical fix mode(3)
    //dataTX[33] = 0x00;    // 0,0,horizontal swing mode(3),horizontal fix mode(3)
    dataTX[34] = 0x00;  // ??
    dataTX[35] = 0x00;  // ??
    dataTX[36] = 0x00;  // ??
    dataTX[37] = 0xFF;  // Checksum
    dataTX[37] = tclacClimate::getChecksum(dataTX, sizeof(dataTX));

    tclacClimate::sendData(dataTX, sizeof(dataTX));
    allow_take_control = false;
    is_call_control = false;
}

// Sending data to AC.
// Reliability strategy on a line without a level converter (3.3V -> 5V UART):
//  1) "Listen before talk": do not transmit while AC itself is transmitting
//     (or while we wait for its poll response) — AC controller is de facto
//     half-duplex and drops frames received during its own transmission.
//  2) TX_REPEAT frame retries with TX_REPEAT_SPACING_MS pause — each retry
//     is a separate fair attempt (back-to-back queue gets swallowed as single frame).
// Frames are idempotent: AC will apply the first correctly received one.
// All non-blocking (set_timeout), loop() is not frozen.
void tclacClimate::sendData(uint8_t * message, uint8_t size) {
    tclacClimate::dataShow(1,1);
    this->tx_size_ = size;  // message always points to dataTX (class member)
    for (uint8_t k = 0; k < TX_REPEAT; k++) {
        if (k == 0) {
            this->try_send_frame_(0, TX_MAX_DEFERS);
        } else {
            this->set_timeout(k * TX_REPEAT_SPACING_MS, [this, k]() {
                this->try_send_frame_(k, TX_MAX_DEFERS);
            });
        }
    }
    ESP_LOGD("TCL", "Message to TCL queued (x%d, spacing %dms)", TX_REPEAT, TX_REPEAT_SPACING_MS);
    tclacClimate::dataShow(1,0);
}

// Check if bus is quiet for transmission
bool tclacClimate::bus_quiet_() {
    const uint32_t now = millis();
    // currently receiving
    if (esphome::uart::UARTDevice::available() > 0)
        return false;
    // received just now — frame might continue
    if (now - this->last_rx_ms_ < BUS_QUIET_MS)
        return false;
    // sent a poll and response hasn't started arriving — don't interfere
    if (now - this->poll_sent_ms_ < POLL_RESPONSE_WINDOW_MS
        && (int32_t)(this->last_rx_ms_ - this->poll_sent_ms_) < 0)
        return false;
    return true;
}

// Send frame if bus is quiet; otherwise defer by BUS_QUIET_MS
void tclacClimate::try_send_frame_(uint8_t attempt, uint8_t defers_left) {
    if (!this->bus_quiet_() && defers_left > 0) {
        this->set_timeout(BUS_QUIET_MS,
            [this, attempt, defers_left]() {
                this->try_send_frame_(attempt, defers_left - 1);
            });
        return;
    }
    this->esphome::uart::UARTDevice::write_array(this->dataTX, this->tx_size_);
    this->esphome::uart::UARTDevice::flush();
}

// Convert byte array to human-readable format.
// Uses std::string instead of Arduino String,
// so function works both with Arduino and ESP-IDF.
std::string tclacClimate::getHex(const uint8_t *message, size_t size) {
    std::string raw;

    for (size_t i = 0; i < size; i++) {
        raw += "\n";
        raw += std::to_string(message[i]);
    }

    return raw;
}
// Checksum calculation
uint8_t tclacClimate::getChecksum(const uint8_t * message, size_t size) {
    uint8_t position = size - 1;
    uint8_t crc = 0;
    for (int i = 0; i < position; i++)
        crc ^= message[i];
    return crc;
}

// Toggle LEDs
void tclacClimate::dataShow(bool flow, bool shine) {
    if (module_display_status_){
        if (flow == 0){
            if (shine == 1){
#ifdef CONF_RX_LED
                this->rx_led_pin_->digital_write(true);
#endif
            } else {
#ifdef CONF_RX_LED
                this->rx_led_pin_->digital_write(false);
#endif
            }
        }
        if (flow == 1) {
            if (shine == 1){
#ifdef CONF_TX_LED
                this->tx_led_pin_->digital_write(true);
#endif
            } else {
#ifdef CONF_TX_LED
                this->tx_led_pin_->digital_write(false);
#endif
            }
        }
    }
}

// Actions with config data

// Get beeper state
void tclacClimate::set_beeper_state(bool state) {
    this->beeper_status_ = state;
    if (force_mode_status_){
        if (allow_take_control){
            tclacClimate::takeControl();
        }
    }
}
// Get AC display state
void tclacClimate::set_display_state(bool disp_state) {
    this->display_status_ = disp_state;
    if (force_mode_status_){
        if (allow_take_control){
            tclacClimate::takeControl();
        }
    }
}
// 
void tclacClimate::set_anti_mildew_state(bool mildew_state) {
    this->anti_mildew_status_ = mildew_state;
    if (force_mode_status_) {
        if (allow_take_control) {
            tclacClimate::takeControl();
        }
    }
}
// Get force mode settings state
void tclacClimate::set_force_mode_state(bool f_state) {
    this->force_mode_status_ = f_state;
}
// Get RX LED pin
#ifdef CONF_RX_LED
void tclacClimate::set_rx_led_pin(GPIOPin *rx_led_pin) {
    this->rx_led_pin_ = rx_led_pin;
}
#endif
// Get TX LED pin
#ifdef CONF_TX_LED
void tclacClimate::set_tx_led_pin(GPIOPin *tx_led_pin) {
    this->tx_led_pin_ = tx_led_pin;
}
#endif
// Get module communication LED state
void tclacClimate::set_module_display_state(bool d_state) {
    this->module_display_status_ = d_state;
}
// Get vertical louver fixed position mode
void tclacClimate::set_vertical_airflow(AirflowVerticalDirection v_airflow) {
    this->vertical_direction_ = v_airflow;
    if (force_mode_status_){
        if (allow_take_control){
            tclacClimate::takeControl();
        }
    }
}
// Get horizontal louvers fixed position mode
void tclacClimate::set_horizontal_airflow(AirflowHorizontalDirection h_airflow) {
    this->horizontal_direction_ = h_airflow;
    if (force_mode_status_){
        if (allow_take_control){
            tclacClimate::takeControl();
        }
    }
}
// Get vertical louver swing direction mode
void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection vs_direction) {
    this->vertical_swing_direction_ = vs_direction;
    if (force_mode_status_){
        if (allow_take_control){
            tclacClimate::takeControl();
        }
    }
}
// Get supported AC operating modes
void tclacClimate::set_supported_modes(climate::ClimateModeMask modes) {
    this->supported_modes_ = modes;
    ESP_LOGD("TCL", "Set up Modes");
}
// Get horizontal louvers swing direction mode
void tclacClimate::set_horizontal_swing_direction(HorizontalSwingDirection hs_direction) {
    horizontal_swing_direction_ = hs_direction;
    if (force_mode_status_){
        if (allow_take_control){
            tclacClimate::takeControl();
        }
    }
}
// Get available fan speeds
void tclacClimate::set_supported_fan_modes(climate::ClimateFanModeMask fan_modes){
	this->supported_fan_modes_ = fan_modes;
}
// Get available damper swing modes
void tclacClimate::set_supported_swing_modes(climate::ClimateSwingModeMask swing_modes) {
	this->supported_swing_modes_ = swing_modes;
}
// Get available presets
void tclacClimate::set_supported_presets(climate::ClimatePresetMask presets) {
  this->supported_presets_ = presets;
}


}
}