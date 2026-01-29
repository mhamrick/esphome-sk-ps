#include "XY-SKxxx-internal.h"
#include "XY-SKxxx.h"

/* Cached value access methods for output measurements */
float XY_SKxxx::getOutputVoltage(bool refresh) {
  if (refresh) {
    updateOutputStatus(true);
  }
  return _status.outputVoltage;
}

float XY_SKxxx::getOutputCurrent(bool refresh) {
  if (refresh) {
    updateOutputStatus(true);
  }
  return _status.outputCurrent;
}

float XY_SKxxx::getOutputPower(bool refresh) {
  if (refresh) {
    updateOutputStatus(true);
  }
  return _status.outputPower;
}

float XY_SKxxx::getInputVoltage(bool refresh) {
  if (refresh) {
    updateOutputStatus(true);
  }
  return _status.inputVoltage;
}

/* Operation mode indicator methods */
bool XY_SKxxx::isInConstantCurrentMode(bool refresh) {
  if (refresh) {
    updateDeviceState(true);
  }
  return (_status.cvccMode == 1);
}

bool XY_SKxxx::isInConstantVoltageMode(bool refresh) {
  if (refresh) {
    updateDeviceState(true);
  }
  return (_status.cvccMode == 0);
}

/* Protection and status methods */
uint16_t XY_SKxxx::getProtectionStatus(bool refresh) {
  if (refresh) {
    updateDeviceState(true);
  }
  return _status.protectionStatus;
}

/* Combined measurement method for convenience */
bool XY_SKxxx::getMeasurements(float &outVoltage, float &outCurrent, float &outPower, 
                              float &inVoltage, bool refresh) {
  if (refresh) {
    if (!updateOutputStatus(true)) {
      return false;
    }
  }
  
  outVoltage = _status.outputVoltage;
  outCurrent = _status.outputCurrent;
  outPower = _status.outputPower;
  inVoltage = _status.inputVoltage;
  
  return true;
}

/* Complete energy measurement method */
bool XY_SKxxx::getEnergyMeasurements(uint32_t &ampHours, uint32_t &wattHours, 
                                   uint32_t &outputTime, bool refresh) {
  if (refresh) {
    if (!updateEnergyMeters(true)) {
      return false;
    }
  }
  
  ampHours = _status.ampHours;
  wattHours = _status.wattHours;
  outputTime = _status.outputTime;
  
  return true;
}

/* Combined temperature measurement method */
bool XY_SKxxx::getTemperatures(float &internalTemp, float &externalTemp, bool refresh) {
  if (refresh) {
    if (!updateTemperatures(true)) {
      return false;
    }
  }
  
  internalTemp = _status.internalTemp;
  externalTemp = _status.externalTemp;
  
  return true;
}

/* Individual temperature measurement methods */
float XY_SKxxx::getInternalTemperature(bool refresh) {
  if (refresh) {
    updateTemperatures(true);
  }
  return _status.internalTemp;
}

float XY_SKxxx::getExternalTemperature(bool refresh) {
  if (refresh) {
    updateTemperatures(true);
  }
  return _status.externalTemp;
}

/* CV/CC status method */
uint16_t XY_SKxxx::getCVCCState(bool refresh) {
  if (refresh) {
    updateDeviceState(true);
  }
  return _status.cvccMode;
}

/* Unified operating mode method */
OperatingMode XY_SKxxx::getOperatingMode(bool refresh) {
  if (refresh) {
    // Update both device state and constant power settings
    updateDeviceState(true);
    updateConstantPowerSettings(true);
  }
  
  // Check operating modes in priority order:
  // 1. First check if constant power mode is enabled (highest priority)
  if (_status.cpModeEnabled) {
    return MODE_CP;
  }
  // 2. Then check CV/CC mode
  else if (_status.cvccMode == 1) {
    return MODE_CC;
  }
  else {
    return MODE_CV;
  }
}
