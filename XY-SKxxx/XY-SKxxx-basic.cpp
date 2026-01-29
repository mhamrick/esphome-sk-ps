// Include guard to prevent multiple compilation
#ifndef XY_SKXXX_BASIC_IMPL
#define XY_SKXXX_BASIC_IMPL

#include "XY-SKxxx-internal.h"
#include "XY-SKxxx.h"

/* Basic device information */
uint16_t XY_SKxxx::getModel() {
  waitForSilentInterval();
  
  uint8_t result = modbus.readHoldingRegisters(REG_MODEL, 1);
  if (result == modbus.ku8MBSuccess) {
    _lastCommsTime = millis();
    return modbus.getResponseBuffer(0);
  }
  
  _lastCommsTime = millis();
  return 0;
}

uint16_t XY_SKxxx::getVersion() {
  waitForSilentInterval();
  
  uint8_t result = modbus.readHoldingRegisters(REG_VERSION, 1);
  if (result == modbus.ku8MBSuccess) {
    _lastCommsTime = millis();
    return modbus.getResponseBuffer(0);
  }
  
  _lastCommsTime = millis();
  return 0;
}

/* Basic output settings */
bool XY_SKxxx::setVoltage(float voltage) {
  if (voltage >= 0.0f && voltage <= 30.0f) { // Adjust based on your device's specifications
    uint16_t voltageValue = (uint16_t)(voltage * 100);
    waitForSilentInterval();
    uint8_t result = modbus.writeSingleRegister(REG_V_SET, voltageValue);
    _lastCommsTime = millis();
    
    if (result == modbus.ku8MBSuccess) {
      _status.setVoltage = voltage;
      return true;
    }
  }
  return false;
}

bool XY_SKxxx::setCurrent(float current) {
  if (current >= 0.0f && current <= 5.1f) { // Adjust based on your device's specifications
    uint16_t currentValue = (uint16_t)(current * 1000);
    waitForSilentInterval();
    uint8_t result = modbus.writeSingleRegister(REG_I_SET, currentValue);
    _lastCommsTime = millis();
    
    if (result == modbus.ku8MBSuccess) {
      _status.setCurrent = current;
      return true;
    }
  }
  return false;
}

/* Combined measurement method for convenience */
bool XY_SKxxx::getOutput(float &voltage, float &current, float &power) {
  waitForSilentInterval();
  
  uint8_t result = modbus.readHoldingRegisters(REG_VOUT, 3);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    voltage = modbus.getResponseBuffer(0) / 100.0f;
    current = modbus.getResponseBuffer(1) / 1000.0f;
    power = modbus.getResponseBuffer(2) / 100.0f;
    
    // Update cache with new values
    _status.outputVoltage = voltage;
    _status.outputCurrent = current;
    _status.outputPower = power;
    
    _lastOutputUpdate = millis();
    return true;
  }
  
  return false;
}

/* Higher-level convenience methods (moved from control.cpp) */
bool XY_SKxxx::setVoltageAndCurrent(float voltage, float current) {
  bool voltageSuccess = false;
  bool currentSuccess = false;
  
  // First make sure the silent interval is observed
  waitForSilentInterval();
  
  // Set voltage with proper timing
  preTransmission();
  voltageSuccess = setVoltage(voltage);
  postTransmission();
  
  // Wait between commands
  delay(_silentIntervalTime * 3);
  
  // Set current with proper timing
  preTransmission();
  currentSuccess = setCurrent(current);
  postTransmission();
  
  // If either operation failed, try again
  if (!voltageSuccess || !currentSuccess) {
    // If voltage failed, retry
    if (!voltageSuccess) {
      delay(_silentIntervalTime * 3);
      preTransmission();
      voltageSuccess = setVoltage(voltage);
      postTransmission();
    }
    
    // If current failed, retry
    if (!currentSuccess) {
      delay(_silentIntervalTime * 3);
      preTransmission();
      currentSuccess = setCurrent(current);
      postTransmission();
    }
  }
  
  return voltageSuccess && currentSuccess;
}

bool XY_SKxxx::setOutputState(bool on) {
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_ONOFF, on ? 1 : 0);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _status.outputEnabled = on;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::turnOutputOn() {
  waitForSilentInterval();
  
  preTransmission();
  bool success = setOutputState(true);
  postTransmission();
  
  // Retry once if failed
  if (!success) {
    delay(_silentIntervalTime * 3);
    preTransmission();
    success = setOutputState(true);
    postTransmission();
  }
  
  return success;
}

bool XY_SKxxx::turnOutputOff() {
  waitForSilentInterval();
  
  preTransmission();
  bool success = setOutputState(false);
  postTransmission();
  
  // Retry once if failed
  if (!success) {
    delay(_silentIntervalTime * 3);
    preTransmission();
    success = setOutputState(false);
    postTransmission();
  }
  
  return success;
}

bool XY_SKxxx::getOutputStatus(float &voltage, float &current, float &power, bool &isOn) {
  waitForSilentInterval();
  
  preTransmission();
  bool success = getOutput(voltage, current, power);
  postTransmission();
  
  if (success) {
    isOn = (power > 0);
  }
  
  return success;
}

bool XY_SKxxx::setKeyLock(bool lock) {
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_LOCK, lock ? 1 : 0);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _status.keyLocked = lock;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::setConstantVoltage(float voltage) {
  uint16_t voltageValue = (uint16_t)(voltage * 100);
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_CV_SET, voltageValue);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.constantVoltage = voltage;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::setConstantCurrent(float current) {
  uint16_t currentValue = (uint16_t)(current * 1000);
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_CC_SET, currentValue);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.constantCurrent = current;
    return true;
  }
  
  return false;
}

#endif // XY_SKXXX_BASIC_IMPL
