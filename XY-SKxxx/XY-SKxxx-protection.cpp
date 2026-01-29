#include "XY-SKxxx-internal.h"
#include "XY-SKxxx.h"

// Over Voltage Protection (OVP)
bool XY_SKxxx::setOverVoltageProtection(float voltage) {
  uint16_t voltageValue = (uint16_t)(voltage * 100);
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_S_OVP, voltageValue);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.overVoltageProtection = voltage;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::getOverVoltageProtection(float &voltage) {
  voltage = getCachedOverVoltageProtection(true);
  return true;
}

float XY_SKxxx::getCachedOverVoltageProtection(bool refresh) {
  if (refresh) {
    updateVoltageCurrentProtection(true);
  }
  return _protection.overVoltageProtection;
}

// Input Low Voltage Protection (LVP)
bool XY_SKxxx::setLowVoltageProtection(float voltage) {
  uint16_t voltageValue = (uint16_t)(voltage * 100);
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_S_LVP, voltageValue);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.lowVoltageProtection = voltage;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::getLowVoltageProtection(float &voltage) {
  voltage = getCachedLowVoltageProtection(true);
  return true;
}

float XY_SKxxx::getCachedLowVoltageProtection(bool refresh) {
  if (refresh) {
    updateVoltageCurrentProtection(true);
  }
  return _protection.lowVoltageProtection;
}

// Over Current Protection (OCP)
bool XY_SKxxx::setOverCurrentProtection(float current) {
  uint16_t currentValue = (uint16_t)(current * 1000);
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_S_OCP, currentValue);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.overCurrentProtection = current;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::getOverCurrentProtection(float &current) {
  current = getCachedOverCurrentProtection(true);
  return true;
}

float XY_SKxxx::getCachedOverCurrentProtection(bool refresh) {
  if (refresh) {
    updateVoltageCurrentProtection(true);
  }
  return _protection.overCurrentProtection;
}

// Over Power Protection (OPP)
bool XY_SKxxx::setOverPowerProtection(float power) {
  uint16_t value = static_cast<uint16_t>(power * 10.0f); // Use 10 for 1 decimal place
  bool success = writeRegister(REG_S_OPP, value);
  if (success) {
    _protection.overPowerProtection = power;
    _lastPowerProtectionUpdate = millis();
  }
  return success;
}

bool XY_SKxxx::getOverPowerProtection(float &power) {
  uint16_t value;
  bool success = readRegister(REG_S_OPP, value);
  if (success) {
    power = value / 10.0f; // Use 10 for 1 decimal place
    _protection.overPowerProtection = power;
    _lastPowerProtectionUpdate = millis();
  }
  return success;
}

float XY_SKxxx::getCachedOverPowerProtection(bool refresh) {
  if (refresh) {
    updatePowerProtection(true);
  }
  return _protection.overPowerProtection;
}

// High Power Protection Time (OHP Hours and Minutes)
bool XY_SKxxx::setHighPowerProtectionTime(uint16_t hours, uint16_t minutes) {
  waitForSilentInterval();
  
  uint8_t resultHours = modbus.writeSingleRegister(REG_S_OHP_H, hours);
  _lastCommsTime = millis();
  
  if (resultHours != modbus.ku8MBSuccess) {
    return false;
  }
  
  delay(_silentIntervalTime * 2);
  
  uint8_t resultMinutes = modbus.writeSingleRegister(REG_S_OHP_M, minutes);
  _lastCommsTime = millis();
  
  if (resultMinutes == modbus.ku8MBSuccess) {
    _protection.highPowerHours = hours;
    _protection.highPowerMinutes = minutes;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::getHighPowerProtectionTime(uint16_t &hours, uint16_t &minutes) {
  getCachedHighPowerProtectionTime(hours, minutes, true);
  return true;
}

void XY_SKxxx::getCachedHighPowerProtectionTime(uint16_t &hours, uint16_t &minutes, bool refresh) {
  if (refresh) {
    updatePowerProtection(true);
  }
  hours = _protection.highPowerHours;
  minutes = _protection.highPowerMinutes;
}

// Over Amp-Hour Protection (OAH Low and High)
bool XY_SKxxx::setOverAmpHourProtection(uint16_t ampHoursLow, uint16_t ampHoursHigh) {
  waitForSilentInterval();
  
  uint8_t resultLow = modbus.writeSingleRegister(REG_S_OAH_L, ampHoursLow);
  _lastCommsTime = millis();
  
  if (resultLow != modbus.ku8MBSuccess) {
    return false;
  }
  
  delay(_silentIntervalTime * 2);
  
  uint8_t resultHigh = modbus.writeSingleRegister(REG_S_OAH_H, ampHoursHigh);
  _lastCommsTime = millis();
  
  if (resultHigh == modbus.ku8MBSuccess) {
    _protection.overAmpHoursLow = ampHoursLow;
    _protection.overAmpHoursHigh = ampHoursHigh;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::getOverAmpHourProtection(uint16_t &ampHoursLow, uint16_t &ampHoursHigh) {
  getCachedOverAmpHourProtection(ampHoursLow, ampHoursHigh, true);
  return true;
}

void XY_SKxxx::getCachedOverAmpHourProtection(uint16_t &ampHoursLow, uint16_t &ampHoursHigh, bool refresh) {
  if (refresh) {
    updateEnergyProtection(true);
  }
  ampHoursLow = _protection.overAmpHoursLow;
  ampHoursHigh = _protection.overAmpHoursHigh;
}

// Over Watt-Hour Protection (OWH Low and High)
bool XY_SKxxx::setOverWattHourProtection(uint16_t wattHoursLow, uint16_t wattHoursHigh) {
  waitForSilentInterval();
  
  uint8_t resultLow = modbus.writeSingleRegister(REG_S_OWH_L, wattHoursLow);
  _lastCommsTime = millis();
  
  if (resultLow != modbus.ku8MBSuccess) {
    return false;
  }
  
  delay(_silentIntervalTime * 2);
  
  uint8_t resultHigh = modbus.writeSingleRegister(REG_S_OWH_H, wattHoursHigh);
  _lastCommsTime = millis();
  
  if (resultHigh == modbus.ku8MBSuccess) {
    _protection.overWattHoursLow = wattHoursLow;
    _protection.overWattHoursHigh = wattHoursHigh;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::getOverWattHourProtection(uint16_t &wattHoursLow, uint16_t &wattHoursHigh) {
  getCachedOverWattHourProtection(wattHoursLow, wattHoursHigh, true);
  return true;
}

void XY_SKxxx::getCachedOverWattHourProtection(uint16_t &wattHoursLow, uint16_t &wattHoursHigh, bool refresh) {
  if (refresh) {
    updateEnergyProtection(true);
  }
  wattHoursLow = _protection.overWattHoursLow;
  wattHoursHigh = _protection.overWattHoursHigh;
}

// Over Temperature Protection (OTP)
bool XY_SKxxx::setOverTemperatureProtection(float temperature) {
  uint16_t tempValue = (uint16_t)(temperature * 10);
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_S_OTP, tempValue);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.overTemperature = temperature;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::getOverTemperatureProtection(float &temperature) {
  temperature = getCachedOverTemperatureProtection(true);
  return true;
}

float XY_SKxxx::getCachedOverTemperatureProtection(bool refresh) {
  if (refresh) {
    updateTemperatureProtection(true);
  }
  return _protection.overTemperature;
}

// Power-On Initialization Setting (Output on/off on startup)
bool XY_SKxxx::setPowerOnInitialization(bool outputOnAtStartup) {
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(REG_S_INI, outputOnAtStartup ? 1 : 0);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.outputOnAtStartup = outputOnAtStartup;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::getPowerOnInitialization(bool &outputOnAtStartup) {
  outputOnAtStartup = getCachedPowerOnInitialization(true);
  return true;
}

bool XY_SKxxx::getCachedPowerOnInitialization(bool refresh) {
  if (refresh) {
    updateStartupSetting(true);
  }
  return _protection.outputOnAtStartup;
}

// Protection settings cache update methods
bool XY_SKxxx::updateAllProtectionSettings(bool force) {
  bool result = true;
  
  result &= updateConstantVoltageCurrentSettings(force);
  result &= updateVoltageCurrentProtection(force);
  result &= updatePowerProtection(force);
  result &= updateEnergyProtection(force);
  result &= updateTemperatureProtection(force);
  result &= updateStartupSetting(force);
  result &= updateBatteryCutoffCurrent(force);  // Add this line
  
  return result;
}

bool XY_SKxxx::updateConstantVoltageCurrentSettings(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastConstantVCUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  uint8_t result = modbus.readHoldingRegisters(REG_CV_SET, 2);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.constantVoltage = modbus.getResponseBuffer(0) / 100.0f;
    _protection.constantCurrent = modbus.getResponseBuffer(1) / 1000.0f;
    
    _lastConstantVCUpdate = now;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::updateVoltageCurrentProtection(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastVoltageCurrentProtectionUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read low voltage, over voltage, and over current protection values
  uint8_t result = modbus.readHoldingRegisters(REG_S_LVP, 3);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.lowVoltageProtection = modbus.getResponseBuffer(0) / 100.0f;
    _protection.overVoltageProtection = modbus.getResponseBuffer(1) / 100.0f;
    _protection.overCurrentProtection = modbus.getResponseBuffer(2) / 1000.0f;
    
    _lastVoltageCurrentProtectionUpdate = now;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::updatePowerProtection(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastPowerProtectionUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read over power protection and high power protection time
  uint8_t result = modbus.readHoldingRegisters(REG_S_OPP, 3);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.overPowerProtection = modbus.getResponseBuffer(0) / 10.0f; // Use 10 for 1 decimal place
    _protection.highPowerHours = modbus.getResponseBuffer(1);
    _protection.highPowerMinutes = modbus.getResponseBuffer(2);
    
    _lastPowerProtectionUpdate = now;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::updateEnergyProtection(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastEnergyProtectionUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read over amp-hour and over watt-hour protection values
  uint8_t result = modbus.readHoldingRegisters(REG_S_OAH_L, 4);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.overAmpHoursLow = modbus.getResponseBuffer(0);
    _protection.overAmpHoursHigh = modbus.getResponseBuffer(1);
    _protection.overWattHoursLow = modbus.getResponseBuffer(2);
    _protection.overWattHoursHigh = modbus.getResponseBuffer(3);
    
    _lastEnergyProtectionUpdate = now;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::updateTemperatureProtection(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastTempProtectionUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read over temperature protection value
  uint8_t result = modbus.readHoldingRegisters(REG_S_OTP, 1);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    // Don't divide by 10.0f - OTP is stored as a whole number with no decimal places
    _protection.overTemperature = modbus.getResponseBuffer(0);
    
    _lastTempProtectionUpdate = now;
    return true;
  }
  
  return false;
}

bool XY_SKxxx::updateStartupSetting(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastStartupSettingUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read power-on initialization setting
  uint8_t result = modbus.readHoldingRegisters(REG_S_INI, 1);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _protection.outputOnAtStartup = (modbus.getResponseBuffer(0) != 0);
    
    _lastStartupSettingUpdate = now;
    return true;
  }
  
  return false;
}

/* Access to cached constant voltage and constant current values */
float XY_SKxxx::getCachedConstantVoltage(bool refresh) {
  if (refresh) {
    updateConstantVoltageCurrentSettings(true);
  }
  return _protection.constantVoltage;
}

float XY_SKxxx::getCachedConstantCurrent(bool refresh) {
  if (refresh) {
    updateConstantVoltageCurrentSettings(true);
  }
  return _protection.constantCurrent;
}
