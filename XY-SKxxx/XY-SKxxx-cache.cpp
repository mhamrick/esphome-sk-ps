#ifndef XY_SKXXX_CACHE_IMPL
#define XY_SKXXX_CACHE_IMPL

#include "XY-SKxxx-internal.h"
#include "XY-SKxxx.h"

/* Status cache update methods */
bool XY_SKxxx::updateAllStatus(bool force) {
  bool success = true;
  
  // Check if enough time has passed since last update or if forced
  unsigned long now = millis();
  
  // Update all status components
  success &= updateOutputStatus(force);
  delay(_silentIntervalTime * 2);
  
  success &= updateDeviceSettings(force);
  delay(_silentIntervalTime * 2);
  
  success &= updateEnergyMeters(force);
  delay(_silentIntervalTime * 2);
  
  success &= updateTemperatures(force);
  delay(_silentIntervalTime * 2);
  
  success &= updateDeviceState(force);
  delay(_silentIntervalTime * 2);
  
  success &= updateConstantPowerSettings(force);
  
  _cacheValid = success;
  
  return success;
}

bool XY_SKxxx::updateDeviceState(bool force) {
  // Check if update is needed based on timeout or force flag
  unsigned long now = millis();
  if (!force && (now - _lastStateUpdate < _cacheTimeout)) {
    return true;
  }
  
  bool success = true;  // Add success flag to track overall operation success
  waitForSilentInterval();
  
  // Read output state
  uint8_t outputResult = modbus.readHoldingRegisters(REG_ONOFF, 1);
  if (outputResult == modbus.ku8MBSuccess) {
    _status.outputEnabled = (modbus.getResponseBuffer(0) != 0);
  } else {
    _lastCommsTime = millis();
    return false;
  }
  
  delay(_silentIntervalTime * 2);
  
  // Read key lock status
  preTransmission();
  uint8_t lockResult = modbus.readHoldingRegisters(REG_LOCK, 1);
  postTransmission();
  
  if (lockResult == modbus.ku8MBSuccess) {
    _status.keyLocked = (modbus.getResponseBuffer(0) != 0);
  } else {
    success = false;
  }
  
  delay(_silentIntervalTime * 2);
  
  // Read protection status
  preTransmission();
  uint8_t protResult = modbus.readHoldingRegisters(REG_PROTECT, 1);
  postTransmission();
  
  if (protResult == modbus.ku8MBSuccess) {
    _status.protectionStatus = modbus.getResponseBuffer(0);
  } else {
    success = false;
  }
  
  delay(_silentIntervalTime * 2);
  
  // Read CC/CV mode
  preTransmission();
  uint8_t cvccResult = modbus.readHoldingRegisters(REG_CVCC, 1);
  postTransmission();
  
  if (cvccResult == modbus.ku8MBSuccess) {
    _status.cvccMode = modbus.getResponseBuffer(0);
  } else {
    success = false;
  }
  
  delay(_silentIntervalTime * 2);
  
  // Read system status
  preTransmission();
  uint8_t sysResult = modbus.readHoldingRegisters(REG_SYS_STATUS, 1);
  postTransmission();
  
  if (sysResult == modbus.ku8MBSuccess) {
    _status.systemStatus = modbus.getResponseBuffer(0);
  } else {
    success = false;
  }
  
  if (success) {
    _lastStateUpdate = now;
  }
  
  return success;
}

bool XY_SKxxx::updateOutputStatus(bool force) {
  // Check if update is needed based on timeout or force flag
  unsigned long now = millis();
  if (!force && (now - _lastOutputUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read output voltage, current, power, and input voltage
  uint8_t result = modbus.readHoldingRegisters(REG_VOUT, 4);
  if (result == modbus.ku8MBSuccess) {
    _status.outputVoltage = modbus.getResponseBuffer(0) / 100.0f;
    _status.outputCurrent = modbus.getResponseBuffer(1) / 1000.0f;
    _status.outputPower = modbus.getResponseBuffer(2) / 100.0f;
    _status.inputVoltage = modbus.getResponseBuffer(3) / 100.0f;
    
    _lastOutputUpdate = now;
    _cacheValid = true;
    _lastCommsTime = millis();
    return true;
  }
  
  _lastCommsTime = millis();
  return false;
}

bool XY_SKxxx::updateDeviceSettings(bool force) {
  // Check if update is needed based on timeout or force flag
  unsigned long now = millis();
  if (!force && (now - _lastSettingsUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read voltage and current settings
  uint8_t result = modbus.readHoldingRegisters(REG_V_SET, 2);
  if (result == modbus.ku8MBSuccess) {
    _status.setVoltage = modbus.getResponseBuffer(0) / 100.0f;
    _status.setCurrent = modbus.getResponseBuffer(1) / 1000.0f;
    
    // Also read backlight and sleep timeout settings
    delay(_silentIntervalTime * 2);
    result = modbus.readHoldingRegisters(REG_B_LED, 2);
    if (result == modbus.ku8MBSuccess) {
      _status.backlightLevel = modbus.getResponseBuffer(0);
      _status.sleepTimeout = modbus.getResponseBuffer(1);
      
      _lastSettingsUpdate = now;
      _lastCommsTime = millis();
      return true;
    }
  }
  
  _lastCommsTime = millis();
  return false;
}

bool XY_SKxxx::updateEnergyMeters(bool force) {
  // Check if update is needed based on timeout or force flag
  unsigned long now = millis();
  if (!force && (now - _lastEnergyUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read amp-hour counter (low and high registers)
  uint8_t result = modbus.readHoldingRegisters(REG_AH_LOW, 2);
  if (result != modbus.ku8MBSuccess) {
    _lastCommsTime = millis();
    return false;
  }
  
  uint16_t ahLow = modbus.getResponseBuffer(0);
  uint16_t ahHigh = modbus.getResponseBuffer(1);
  _status.ampHours = (uint32_t)ahHigh << 16 | ahLow;
  
  // Read watt-hour counter (low and high registers)
  delay(_silentIntervalTime * 2);
  result = modbus.readHoldingRegisters(REG_WH_LOW, 2);
  if (result != modbus.ku8MBSuccess) {
    _lastCommsTime = millis();
    return false;
  }
  
  uint16_t whLow = modbus.getResponseBuffer(0);
  uint16_t whHigh = modbus.getResponseBuffer(1);
  _status.wattHours = (uint32_t)whHigh << 16 | whLow;
  
  // Read output time (hours, minutes, seconds)
  delay(_silentIntervalTime * 2);
  result = modbus.readHoldingRegisters(REG_OUT_H, 3);
  if (result != modbus.ku8MBSuccess) {
    _lastCommsTime = millis();
    return false;
  }
  
  uint16_t hours = modbus.getResponseBuffer(0);
  uint16_t minutes = modbus.getResponseBuffer(1);
  uint16_t seconds = modbus.getResponseBuffer(2);
  _status.outputTime = hours * 3600 + minutes * 60 + seconds;
  
  _lastEnergyUpdate = now;
  _lastCommsTime = millis();
  return true;
}

bool XY_SKxxx::updateTemperatures(bool force) {
  // Check if update is needed based on timeout or force flag
  unsigned long now = millis();
  if (!force && (now - _lastTempUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read internal and external temperatures
  uint8_t result = modbus.readHoldingRegisters(REG_T_IN, 2);
  if (result == modbus.ku8MBSuccess) {
    _status.internalTemp = modbus.getResponseBuffer(0) / 10.0f;
    _status.externalTemp = modbus.getResponseBuffer(1) / 10.0f;
    
    _lastTempUpdate = now;
    _lastCommsTime = millis();
    return true;
  }
  
  _lastCommsTime = millis();
  return false;
}

bool XY_SKxxx::updateCalibrationSettings(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastCalibrationUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read internal temperature calibration
  uint8_t result = modbus.readHoldingRegisters(REG_T_IN_CAL, 1);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _internalTempCalibration = (int16_t)modbus.getResponseBuffer(0) / 10.0f;
  }
  
  delay(_silentIntervalTime * 2);
  
  // Read external temperature calibration
  result = modbus.readHoldingRegisters(REG_T_EXT_CAL, 1);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _externalTempCalibration = (int16_t)modbus.getResponseBuffer(0) / 10.0f;
  }
  
  delay(_silentIntervalTime * 2);
  
  // Read beeper setting
  result = modbus.readHoldingRegisters(REG_BEEPER, 1);
  _lastCommsTime = millis();
  
  if (result == modbus.ku8MBSuccess) {
    _beeperEnabled = (modbus.getResponseBuffer(0) != 0);
  }
  
  // Read selected data group
  delay(_silentIntervalTime * 2);
  result = modbus.readHoldingRegisters(REG_EXTRACT_M, 1);
  if (result != modbus.ku8MBSuccess) {
    _lastCommsTime = millis();
    return false;
  }
  
  _selectedDataGroup = modbus.getResponseBuffer(0);
  
  // Read MPPT enable state
  delay(_silentIntervalTime * 2);
  result = modbus.readHoldingRegisters(REG_MPPT_ENABLE, 1);
  if (result == modbus.ku8MBSuccess) {
    _mpptEnabled = (modbus.getResponseBuffer(0) != 0);
  }
  
  // Read MPPT threshold
  delay(_silentIntervalTime * 2);
  result = modbus.readHoldingRegisters(REG_MPPT_THRESHOLD, 1);
  if (result == modbus.ku8MBSuccess) {
    _mpptThreshold = modbus.getResponseBuffer(0) / 100.0f;
  }
  
  _lastCalibrationUpdate = now;
  _lastCommsTime = millis();
  return true;
}

// Battery cutoff current methods
bool XY_SKxxx::updateBatteryCutoffCurrent(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastBatteryCutoffUpdate < _cacheTimeout)) {
    return true;
  }

  waitForSilentInterval();
  
  // Read battery cutoff current
  uint8_t result = modbus.readHoldingRegisters(REG_BTF, 1);
  if (result == modbus.ku8MBSuccess) {
    _protection.batteryCutoffCurrent = modbus.getResponseBuffer(0) / 1000.0f; // 3 decimal places
    _lastBatteryCutoffUpdate = now;
    _lastCommsTime = millis();
    return true;
  }
  
  _lastCommsTime = millis();
  return false;
}

float XY_SKxxx::getCachedBatteryCutoffCurrent(bool refresh) {
  if (refresh) {
    updateBatteryCutoffCurrent(true);
  }
  return _protection.batteryCutoffCurrent;
}

// Communication settings (slave address and baudrate)
bool XY_SKxxx::updateCommunicationSettings(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastCommunicationSettingsUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read slave address
  uint8_t result = modbus.readHoldingRegisters(REG_SLAVE_ADDR, 1);
  if (result == modbus.ku8MBSuccess) {
    _cachedSlaveAddress = modbus.getResponseBuffer(0);
    
    // Read baudrate code
    delay(_silentIntervalTime * 2);
    result = modbus.readHoldingRegisters(REG_BAUDRATE_L, 1);
    if (result == modbus.ku8MBSuccess) {
      _cachedBaudRateCode = modbus.getResponseBuffer(0);
      _lastCommunicationSettingsUpdate = now;
      _lastCommsTime = millis();
      return true;
    }
  }
  
  _lastCommsTime = millis();
  return false;
}

// Device state access methods
bool XY_SKxxx::isOutputEnabled(bool refresh) {
  if (refresh) {
    updateDeviceState(true);
  }
  return _status.outputEnabled;
}

bool XY_SKxxx::isKeyLocked(bool refresh) {
  if (refresh) {
    updateDeviceState(true);
  }
  return _status.keyLocked;
}

uint16_t XY_SKxxx::getSystemStatus(bool refresh) {
  if (refresh) {
    updateDeviceState(true);
  }
  return _status.systemStatus;
}

// Device settings access methods
float XY_SKxxx::getSetVoltage(bool refresh) {
  if (refresh) {
    updateDeviceSettings(true);
  }
  return _status.setVoltage;
}

float XY_SKxxx::getSetCurrent(bool refresh) {
  if (refresh) {
    updateDeviceSettings(true);
  }
  return _status.setCurrent;
}

// Calibration settings access methods
float XY_SKxxx::getInternalTempCalibration(bool refresh) {
  if (refresh) {
    updateCalibrationSettings(true);
  }
  return _internalTempCalibration;
}

float XY_SKxxx::getExternalTempCalibration(bool refresh) {
  if (refresh) {
    updateCalibrationSettings(true);
  }
  return _externalTempCalibration;
}

// Constant Power settings update method
bool XY_SKxxx::updateConstantPowerSettings(bool force) {
  unsigned long now = millis();
  if (!force && (now - _lastConstantPowerUpdate < _cacheTimeout)) {
    return true;
  }
  
  waitForSilentInterval();
  
  // Read CP mode enable state
  uint8_t result = modbus.readHoldingRegisters(REG_CP_ENABLE, 1);
  if (result != modbus.ku8MBSuccess) {
    _lastCommsTime = millis();
    return false;
  }
  
  _status.cpModeEnabled = (modbus.getResponseBuffer(0) != 0);
  
  // Read CP value
  delay(_silentIntervalTime * 2);
  result = modbus.readHoldingRegisters(REG_CP_SET, 1);
  if (result != modbus.ku8MBSuccess) {
    _lastCommsTime = millis();
    return false;
  }
  
  _status.constantPower = modbus.getResponseBuffer(0) / 10.0f;
  _lastConstantPowerUpdate = now;
  _lastCommsTime = millis();
  return true;
}

#endif // XY_SKXXX_CACHE_IMPL
