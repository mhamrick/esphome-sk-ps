#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/modbus/modbus.h"

// Register definitions for XY-SKxxx series (copied from XY-SKxxx.h)
// Define Modbus register addresses (follow protocol naming convention)
#define REG_V_SET 0x0000        // Voltage setting, 2 bytes, 2 decimal places, unit: V, Read and Write
#define REG_I_SET 0x0001        // Current setting, 2 bytes, 3 decimal places, unit: A, Read and Write

#define REG_VOUT 0x0002         // Output voltage display value, 2 bytes, 2 decimal places, unit: V, Read only
#define REG_IOUT 0x0003         // Output current display value, 2 bytes, 3 decimal places, unit: A, Read only

#define REG_POWER 0x0004        // Output power display value, 2 bytes, 2 decimal places, unit: W, Read only
#define REG_UIN 0x0005          // Input voltage display value, 2 bytes, 2 decimal places, unit: V, Read only

#define REG_AH_LOW 0x0006       // Amp-hour low register, 2 bytes, 0 decimal places, unit: mAh, Read only
#define REG_AH_HIGH 0x0007      // Amp-hour high register, 2 bytes, 0 decimal places, unit: mAh, Read only

#define REG_WH_LOW 0x0008       // Watt-hour low register, 2 bytes, 0 decimal places, unit: mWh, Read only
#define REG_WH_HIGH 0x0009      // Watt-hour high register, 2 bytes, 0 decimal places, unit: mWh, Read only

#define REG_OUT_H 0x000A        // Hours of output time, 2 bytes, 0 decimal places, unit: h, Read only
#define REG_OUT_M 0x000B        // Minutes of output time, 2 bytes, 0 decimal places, unit: min, Read only
#define REG_OUT_S 0x000C        // Seconds of output time, 2 bytes, 0 decimal places, unit: s, Read only

#define REG_T_IN 0x000D         // Internal temperature, 2 bytes, 1 decimal place, unit: °F / °C, Read only
#define REG_T_EX 0x000E         // External temperature, 2 bytes, 1 decimal place, unit: °F / °C, Read only

#define REG_LOCK 0x000F         // Key lock status, 2 bytes, 0 decimal places, unit: 0/1 (0: unlock, 1: lock), Read and Write
#define REG_PROTECT 0x0010      // WIP: Protection status???, 2 bytes, 0 decimal places, unit: 0/1, Read and Write - XY6020L-Modbus-Interface.pdf: note 3

#define REG_CVCC 0x0011         // CC/CV (constant current / constant voltage) mode status, 2 bytes, 0 decimal places, unit: 0/1, Read only

#define REG_ONOFF 0x0012        // Output on/off status, 2 bytes, 0 decimal places, unit: 0/1, Read and Write

#define REG_F_C 0x0013          // temperature unit, 2 bytes, 0 decimal places, unit: 0=Celsius, 1=Fahrenheit, Read and Write

#define REG_B_LED 0x0014        // WIP: Backlight brightness, 2 bytes, 0 decimal places, unit: 0-5, Read and Write, factory default: 5 (brightest)

#define REG_SLEEP 0x0015        // Sleep timeout, 2 bytes, 0 decimal places, unit: min, Read and Write, factory default: 2 min

#define REG_MODEL 0x0016        // WIP: Model number: XY-SK120 returns 22873, 2 bytes, 0 decimal places, unit: ???, Read only
#define REG_VERSION 0x0017      // DPS Firmware Version number, 2 bytes, 0 decimal places, unit: ???, Read only

#define REG_SLAVE_ADDR 0x0018   // Modbus Slave address, 2 bytes, 0 decimal places, unit: 0-247, Read and Write, factory default: 1
#define REG_BAUDRATE_L 0x0019   // Baud rate setting, 2 bytes, 0 decimal places, unit: 0-8 (0: 9600, 1: 14400, 2: 19200, 3: 38400, 4: 56000, 5: 576000, 6: 115200, 7: 2400, 8: 4800), Read and Write, factory default: 6 (115200)

#define REG_T_IN_CAL 0x001A     // Internal temperature calibration, 2 bytes, 1 decimal place, unit: °F / °C, Read and Write
#define REG_T_EXT_CAL 0x001B    // External temperature calibration, 2 bytes, 1 decimal place, unit: °F / °C, Read and Write

#define REG_EXTRACT_M 0x001D    // Data group selection, 2 bytes, 0 decimal places, unit: 0-9, Read and Write

#define REG_SYS_STATUS 0x001E   // WIP: System status???, 2 bytes, 0 decimal places, unit: ???, Read and Write

// Additional Modbus register addresses for XY-SK120 0x0030 - 0x0034, will not implement these as it's related to the Sinilink ESP8285H16 module

// Additional Modbus register addresses for XY-SK120 0x0050 - 0x005D
#define REG_CV_SET         0x0050    // CV (constant voltage) setting, 2 bytes, 2 decimal places, unit: V, Read and Write
#define REG_CC_SET         0x0051    // CC (constant current) setting, 2 bytes, 3 decimal places, unit: A, Read and Write

#define REG_S_LVP          0x0052    // LVP (input low voltage protection) setting, 2 bytes, 2 decimal places, unit: V, Read and Write
#define REG_S_OVP          0x0053    // OVP (output over voltage protection) setting, 2 bytes, 2 decimal places, unit: V, Read and Write

#define REG_S_OCP          0x0054    // OCP (output over current protection) setting, 2 bytes, 3 decimal places, unit: A, Read and Write
#define REG_S_OPP          0x0055    // OPP (output over power protection) setting, 2 bytes, 1 decimal place, unit: W, Read and Write

#define REG_S_OHP_H        0x0056    // OHP_H (output high power protection - hours) setting, 2 bytes, 0 decimal places, unit: h, Read and Write
#define REG_S_OHP_M        0x0057    // OHP_M (output high power protection - minutes) setting, 2 bytes, 0 decimal places, unit: min, Read and Write

#define REG_S_OAH_L        0x0058    // OAH_LOW (over-amp-hour protection - low) setting, 2 bytes, 0 decimal places, unit: mAh, Read and Write
#define REG_S_OAH_H        0x0059    // OAH_HIGH (over-amp-hour protection - high) setting, 2 bytes, 0 decimal places, unit: mAh, Read and Write

#define REG_S_OWH_L        0x005A    // OWH_LOW (over-watt-hour protection - low) setting, 2 bytes, 0 decimal places, unit: 10mWh, Read and Write
#define REG_S_OWH_H        0x005B    // OWH_HIGH (over-watt-hour protection - high) setting, 2 bytes, 0 decimal places, unit: 10mWh, Read and Write

#define REG_S_OTP          0x005C   // Over temperature protection setting, 2 bytes, 1 decimal place, unit: °F / °C, Read and Write

#define REG_S_INI          0x005D    // Power-on initialization setting, 2 bytes, 0 decimal places, unit: 0/1 (0: output off upon power on, 1: output on upon power on), Read and Write

// Additional Modbus register addresses for XY-SK120 0x0100 - 0x0103, will not implement these as it's related to RTC (Real Time Clock) settings

// Additional Modbus register addresses for XY-SK120 0x0110 - 0x011D, will not implement these as it's related to weather infomration???

/* Below are undocumented registers, available in the XY-SK120 manual and OSD (On-Screen Display) 
but not in the Modbus register map documentation
*/

#define REG_S_ETP           0x005E    // WIP: Not implemented.
// REG_S_ETP: both 0x005E and 0x005F stores the ETP (External Temperature Protection) value, but 0x005E is read-write and 0x005F is read-only and both values seems to be mirrored. 
// However, writing to 0x005E does not seem to have any effect on the device, so it's likely not implemented or used.

// beeper settings (beeper enable)
#define REG_BEEPER          0x001C       // Beeper enable/disable, 2 bytes, 0 decimal places, unit: 0/1, Read and Write

// RET setting (Restore factory settings) (discovered through testing)
#define REG_FACTORY_RESET   0x0025 // Factory reset, write 0x1 to trigger reset to defaults

// FET setting (quick adjustment of voltage, current or power with the rotary encoder knob)
// Cannot probe the register for FET setting despite the best effort, so it's not implemented

// PPT Setting (MPPT Solar Charging Settings)
#define REG_MPPT_ENABLE     0x001F  // MPPT enable/disable, 2 bytes, 0 decimal places, unit: 0/1, Read and Write
#define REG_MPPT_THRESHOLD  0x0020 // MPPT threshold percentage, 2 bytes, 2 decimal places, unit: ratio (0.00-1.00), Read and Write

// BTF setting (Battery Full)
#define REG_BTF             0x0021 // Battery charge cut off current, 2 bytes, 3 decimal places, unit: A, Read and Write, set 0 to turn off

// CP Setting (Constant Power Mode)
#define REG_CP_ENABLE     0x0022  // Constant Power mode enable/disable, 2 bytes, 0 decimal places, unit: 0/1, Read and Write
#define REG_CP_SET        0x0023  // Constant Power setting, 2 bytes, 1 decimal place, unit: W, Read and Write

// Number of registers to read in the device status request
#define REG_STATUS_COUNT  0x0014  // Read 20 registers (0x0014) starting at REG_V_SET

namespace esphome {
namespace xysk {

enum CurrentResolution {
  XYSK_CURRENT_RESOLUTION_AUTO,
  XYSK_CURRENT_RESOLUTION_LOW,
  XYSK_CURRENT_RESOLUTION_HIGH,
};

class Xysk : public PollingComponent, public modbus::ModbusDevice {
 public:
  void set_output_binary_sensor(binary_sensor::BinarySensor *output_binary_sensor) {
    output_binary_sensor_ = output_binary_sensor;
  }
  void set_key_lock_binary_sensor(binary_sensor::BinarySensor *key_lock_binary_sensor) {
    key_lock_binary_sensor_ = key_lock_binary_sensor;
  }
  void set_constant_current_mode_binary_sensor(binary_sensor::BinarySensor *constant_current_mode_binary_sensor) {
    constant_current_mode_binary_sensor_ = constant_current_mode_binary_sensor;
  }

  void set_voltage_setting_number(number::Number *voltage_setting_number) {
    voltage_setting_number_ = voltage_setting_number;
  }
  void set_current_setting_number(number::Number *current_setting_number) {
    current_setting_number_ = current_setting_number;
  }

  void set_output_voltage_sensor(sensor::Sensor *output_voltage_sensor) {
    output_voltage_sensor_ = output_voltage_sensor;
  }
  void set_output_current_sensor(sensor::Sensor *output_current_sensor) {
    output_current_sensor_ = output_current_sensor;
  }
  void set_output_power_sensor(sensor::Sensor *output_power_sensor) { output_power_sensor_ = output_power_sensor; }
  void set_input_voltage_sensor(sensor::Sensor *input_voltage_sensor) { input_voltage_sensor_ = input_voltage_sensor; }
  void set_voltage_setting_sensor(sensor::Sensor *voltage_setting_sensor) {
    voltage_setting_sensor_ = voltage_setting_sensor;
  }
  void set_current_setting_sensor(sensor::Sensor *current_setting_sensor) {
    current_setting_sensor_ = current_setting_sensor;
  }
  void set_backlight_brightness_sensor(sensor::Sensor *backlight_brightness_sensor) {
    backlight_brightness_sensor_ = backlight_brightness_sensor;
  }
  void set_firmware_version_sensor(sensor::Sensor *firmware_version_sensor) {
    firmware_version_sensor_ = firmware_version_sensor;
  }
  void set_internal_temperature_sensor(sensor::Sensor *internal_temperature_sensor) {
    internal_temperature_sensor_ = internal_temperature_sensor;
  }

  void set_output_switch(switch_::Switch *output_switch) { output_switch_ = output_switch; }
  void set_key_lock_switch(switch_::Switch *key_lock_switch) { key_lock_switch_ = key_lock_switch; }

  void set_protection_status_text_sensor(text_sensor::TextSensor *protection_status_text_sensor) {
    protection_status_text_sensor_ = protection_status_text_sensor;
  }
  void set_device_model_text_sensor(text_sensor::TextSensor *device_model_text_sensor) {
    device_model_text_sensor_ = device_model_text_sensor;
  }
  void set_current_resolution(CurrentResolution current_resolution) { current_resolution_ = current_resolution; }
  void set_current_resolution_if_auto(CurrentResolution current_resolution) {
    if (this->current_resolution_ == XYSK_CURRENT_RESOLUTION_AUTO) {
      this->set_current_resolution(current_resolution);
    }
  }
  float current_resolution_factor() {
    return (this->current_resolution_ == XYSK_CURRENT_RESOLUTION_HIGH) ? 0.001f : 0.01f;
  }

  void dump_config() override;

  void on_modbus_data(const std::vector<uint8_t> &data) override;

  void update() override;

  void write_register(uint16_t address, uint16_t value);

 protected:
  CurrentResolution current_resolution_{XYSK_CURRENT_RESOLUTION_AUTO};

  binary_sensor::BinarySensor *output_binary_sensor_;
  binary_sensor::BinarySensor *key_lock_binary_sensor_;
  binary_sensor::BinarySensor *constant_current_mode_binary_sensor_;

  number::Number *voltage_setting_number_;
  number::Number *current_setting_number_;

  sensor::Sensor *output_voltage_sensor_;
  sensor::Sensor *output_current_sensor_;
  sensor::Sensor *output_power_sensor_;
  sensor::Sensor *input_voltage_sensor_;
  sensor::Sensor *voltage_setting_sensor_;
  sensor::Sensor *current_setting_sensor_;
  sensor::Sensor *backlight_brightness_sensor_;
  sensor::Sensor *firmware_version_sensor_;
  sensor::Sensor *internal_temperature_sensor_;

  switch_::Switch *output_switch_;
  switch_::Switch *key_lock_switch_;

  text_sensor::TextSensor *protection_status_text_sensor_;
  text_sensor::TextSensor *device_model_text_sensor_;

  void on_status_data_(const std::vector<uint8_t> &data);
  void on_acknowledge_data_(const std::vector<uint8_t> &data);
  void publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state);
  void publish_state_(number::Number *number, float value);
  void publish_state_(sensor::Sensor *sensor, float value);
  void publish_state_(switch_::Switch *obj, const bool &state);
  void publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state);
};

}  // namespace xysk
}  // namespace esphome
