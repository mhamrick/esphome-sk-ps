#include "XY-SKxxx.h"

// Update beeper enable/disable
bool XY_SKxxx::setBeeper(bool enabled) {
  uint16_t value = enabled ? 1 : 0;
  bool success = writeRegister(REG_BEEPER, value);
  
  if (success) {
    _beeperEnabled = enabled;
  }
  
  return success;
}

bool XY_SKxxx::getBeeper(bool &enabled) {
  uint16_t value;
  bool success = readRegister(REG_BEEPER, value);
  
  if (success) {
    enabled = (value != 0);
    _beeperEnabled = enabled;
  }
  
  return success;
}