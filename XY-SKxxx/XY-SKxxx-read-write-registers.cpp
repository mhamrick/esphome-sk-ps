#include "XY-SKxxx-internal.h"
#include "XY-SKxxx.h"

/**
 * Debug function to read directly from a register
 * 
 * @param addr Register address to read from
 * @param count Number of registers to read (1-125)
 * @param values Array to store the read values (must be at least 'count' in size)
 * @return true if successful
 */
bool XY_SKxxx::debugReadRegisters(uint16_t addr, uint8_t count, uint16_t* values) {
  if (count < 1 || count > 125) {
    return false; // Invalid count value
  }
  
  waitForSilentInterval();
  
  // Try to read holding registers first
  uint8_t result = modbus.readHoldingRegisters(addr, count);
  
  if (result == modbus.ku8MBSuccess) {
    // Copy the results to the output array
    for (uint8_t i = 0; i < count; i++) {
      values[i] = modbus.getResponseBuffer(i);
    }
    _lastCommsTime = millis();
    return true;
  }
  
  // If holding registers fail, try input registers
  delay(_silentIntervalTime * 2);
  result = modbus.readInputRegisters(addr, count);
  
  if (result == modbus.ku8MBSuccess) {
    // Copy the results to the output array
    for (uint8_t i = 0; i < count; i++) {
      values[i] = modbus.getResponseBuffer(i);
    }
    _lastCommsTime = millis();
    return true;
  }
  
  _lastCommsTime = millis();
  return false;
}

/**
 * Debug function to write directly to a register
 * 
 * @param addr Register address to write to
 * @param value Value to write
 * @return true if successful
 */
bool XY_SKxxx::debugWriteRegister(uint16_t addr, uint16_t value) {
  waitForSilentInterval();
  
  uint8_t result = modbus.writeSingleRegister(addr, value);
  _lastCommsTime = millis();
  
  return (result == modbus.ku8MBSuccess);
}

/**
 * Debug function to write to multiple consecutive registers
 * 
 * @param addr Starting register address
 * @param count Number of registers to write (1-123)
 * @param values Array of values to write (must be at least 'count' in size)
 * @return true if successful
 */
bool XY_SKxxx::debugWriteRegisters(uint16_t addr, uint8_t count, const uint16_t* values) {
  if (count < 1 || count > 123) {
    return false; // Invalid count value
  }
  
  waitForSilentInterval();
  
  // Set the transmit buffer with the values to write
  for (uint8_t i = 0; i < count; i++) {
    modbus.setTransmitBuffer(i, values[i]);
  }
  
  // Write the values to the registers
  uint8_t result = modbus.writeMultipleRegisters(addr, count);
  _lastCommsTime = millis();
  
  return (result == modbus.ku8MBSuccess);
}
