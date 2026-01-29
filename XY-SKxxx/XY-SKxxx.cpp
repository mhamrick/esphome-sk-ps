#include "XY-SKxxx-internal.h" // Include internal header
#include "XY-SKxxx-cd-data-group.h" // Add include for the memory group header

/* Initialize static member */
XY_SKxxx* XY_SKxxx::_instance = nullptr;

XY_SKxxx::XY_SKxxx(uint8_t rxPin, uint8_t txPin, uint8_t slaveID)
  : _rxPin(rxPin), _txPin(txPin), _slaveID(slaveID), _lastCommsTime(0), _silentIntervalTime(0),
    _lastOutputUpdate(0), _lastSettingsUpdate(0), _lastEnergyUpdate(0), _lastTempUpdate(0), 
    _lastStateUpdate(0), _lastConstantVCUpdate(0), _lastVoltageCurrentProtectionUpdate(0),
    _lastPowerProtectionUpdate(0), _lastEnergyProtectionUpdate(0), _lastTempProtectionUpdate(0),
    _lastStartupSettingUpdate(0), _lastBatteryCutoffUpdate(0), _lastCommunicationSettingsUpdate(0),
    _lastConstantPowerUpdate(0), _cacheTimeout(5000), _cacheValid(false) {
  // Store instance pointer for static callback use
  _instance = this;
  
  // Initialize device status with default values
  memset(&_status, 0, sizeof(DeviceStatus));
  memset(&_protection, 0, sizeof(ProtectionSettings)); 
  
  // Initialize memory group cache
  for (int i = 0; i < 10; i++) {
    groupCache[i].valid = false;
    groupCache[i].lastUpdate = 0;
  }
  
  // Initialize communication settings cache
  _cachedSlaveAddress = slaveID;
  _cachedBaudRateCode = 6; // Default is 115200 (code 6)
}

void XY_SKxxx::begin(long baudRate) {
  _baudRate = baudRate;
  _silentIntervalTime = silentInterval(baudRate);
  
  // Initialize hardware serial for XIAO ESP32S3
  Serial1.begin(baudRate, SERIAL_8N1, _rxPin, _txPin);
  
  // Initialize ModbusMaster with Serial1
  modbus.begin(_slaveID, Serial1);
  
  // Set up pre and post transmission callbacks using static functions
  modbus.preTransmission(staticPreTransmission);
  modbus.postTransmission(staticPostTransmission);
}

/* Modbus RTU timing methods */
unsigned long XY_SKxxx::silentInterval(unsigned long baudRate) {
  // 3.5 character times = 3.5 * (11 bits/character)
  // 11 bits = 1 start bit + 8 data bits + 1 parity bit + 1 stop bit in Modbus-RTU asynchronous transmission
  float characterTime = 1000.0 / (float)(baudRate / 11.0); // Milliseconds per character
  return (unsigned long)(3.5 * characterTime);
}

void XY_SKxxx::waitForSilentInterval() {
  unsigned long elapsed = millis() - _lastCommsTime;
  // Use a longer wait time to ensure device is ready
  if (elapsed < (_silentIntervalTime * 2) && _lastCommsTime > 0) {
    // Need to wait for the remaining silent interval time
    delay((_silentIntervalTime * 2) - elapsed);
  }
}

bool XY_SKxxx::preTransmission() {
  waitForSilentInterval();
  return true;
}

bool XY_SKxxx::postTransmission() {
  _lastCommsTime = millis();
  return true;
}

void XY_SKxxx::staticPreTransmission() {
  if (_instance) {
    _instance->preTransmission();
  }
}

void XY_SKxxx::staticPostTransmission() {
  if (_instance) {
    _instance->postTransmission();
  }
}

bool XY_SKxxx::testConnection() {
  waitForSilentInterval();
  preTransmission();
  uint16_t model = getModel();
  postTransmission();
  return model > 0; // Return true if we got a valid model number
}

// Direct register access methods for memory groups
bool XY_SKxxx::readRegisters(uint16_t addr, uint16_t count, uint16_t* buffer) {
  waitForSilentInterval();
  uint8_t result = modbus.readHoldingRegisters(addr, count);
  if (result == modbus.ku8MBSuccess) {
    for (uint16_t i = 0; i < count; i++) {
      buffer[i] = modbus.getResponseBuffer(i);
    }
    _lastCommsTime = millis();
    return true;
  }
  _lastCommsTime = millis();
  return false;
}

bool XY_SKxxx::writeRegister(uint16_t addr, uint16_t value) {
  waitForSilentInterval();
  uint8_t result = modbus.writeSingleRegister(addr, value);
  _lastCommsTime = millis();
  return (result == modbus.ku8MBSuccess);
}

bool XY_SKxxx::writeRegisters(uint16_t addr, uint16_t count, uint16_t* buffer) {
  waitForSilentInterval();
  for (uint16_t i = 0; i < count; i++) {
    modbus.setTransmitBuffer(i, buffer[i]);
  }
  uint8_t result = modbus.writeMultipleRegisters(addr, count);
  _lastCommsTime = millis();
  return (result == modbus.ku8MBSuccess);
}

// Add a single register read method
bool XY_SKxxx::readRegister(uint16_t addr, uint16_t& value) {
  waitForSilentInterval();
  uint16_t buffer[1];
  bool success = readRegisters(addr, 1, buffer);
  if (success) {
    value = buffer[0];
  }
  return success;
}

// Add memory group methods implementation
bool XY_SKxxx::readMemoryGroup(xy_sk::MemoryGroup group, uint16_t* data, bool force) {
    // Try to get from cache first unless forced refresh is requested
    if (!force && getCachedMemoryGroup(group, data, false)) {
        return true;
    }
    
    // Cache miss or forced refresh - read from device
    uint16_t startAddr = xy_sk::DataGroupManager::getGroupStartAddress(group);
    bool success = readRegisters(startAddr, xy_sk::DATA_GROUP_REGISTERS, data);
    
    // Update cache if read was successful
    if (success) {
        uint8_t groupIdx = static_cast<uint8_t>(group);
        memcpy(groupCache[groupIdx].values, data, xy_sk::DATA_GROUP_REGISTERS * sizeof(uint16_t));
        groupCache[groupIdx].valid = true;
        groupCache[groupIdx].lastUpdate = millis();
    }
    
    return success;
}

bool XY_SKxxx::writeMemoryGroup(xy_sk::MemoryGroup group, const uint16_t* data) {
    uint16_t startAddr = xy_sk::DataGroupManager::getGroupStartAddress(group);
    bool success = writeRegisters(startAddr, xy_sk::DATA_GROUP_REGISTERS, const_cast<uint16_t*>(data));
    
    // Update cache if write was successful
    if (success) {
        uint8_t groupIdx = static_cast<uint8_t>(group);
        memcpy(groupCache[groupIdx].values, data, xy_sk::DATA_GROUP_REGISTERS * sizeof(uint16_t));
        groupCache[groupIdx].valid = true;
        groupCache[groupIdx].lastUpdate = millis();
    }
    
    return success;
}

bool XY_SKxxx::callMemoryGroup(xy_sk::MemoryGroup group) {
    // M0 is already active, no need to call it
    if (group == xy_sk::MemoryGroup::M0) {
        return true;
    }
    
    // Write memory group number to EXTRACT_M register (001DH)
    bool success = writeRegister(xy_sk::EXTRACT_M_REGISTER, static_cast<uint16_t>(group));
    
    // If successful, invalidate M0 cache as it will now contain data from the called group
    if (success) {
        groupCache[0].valid = false;
    }
    
    return success;
}

bool XY_SKxxx::readGroupRegister(xy_sk::MemoryGroup group, xy_sk::GroupRegisterOffset regOffset, uint16_t& value) {
    // Try to get from cache first
    uint8_t groupIdx = static_cast<uint8_t>(group);
    uint8_t offsetIdx = static_cast<uint8_t>(regOffset);
    
    if (groupCache[groupIdx].valid && offsetIdx < xy_sk::DATA_GROUP_REGISTERS) {
        value = groupCache[groupIdx].values[offsetIdx];
        return true;
    }
    
    // Cache miss - read individual register from device
    uint16_t addr = xy_sk::DataGroupManager::getRegisterAddress(group, regOffset);
    return readRegister(addr, value);
}

bool XY_SKxxx::writeGroupRegister(xy_sk::MemoryGroup group, xy_sk::GroupRegisterOffset regOffset, uint16_t value) {
    uint16_t addr = xy_sk::DataGroupManager::getRegisterAddress(group, regOffset);
    bool success = writeRegister(addr, value);
    
    // Update cache if write was successful
    if (success) {
        uint8_t groupIdx = static_cast<uint8_t>(group);
        uint8_t offsetIdx = static_cast<uint8_t>(regOffset);
        
        if (groupCache[groupIdx].valid && offsetIdx < xy_sk::DATA_GROUP_REGISTERS) {
            groupCache[groupIdx].values[offsetIdx] = value;
        } else {
            // Invalidate cache entry if it wasn't already valid
            groupCache[groupIdx].valid = false;
        }
    }
    
    return success;
}

bool XY_SKxxx::getCachedMemoryGroup(xy_sk::MemoryGroup group, uint16_t* data, bool refresh) {
    uint8_t groupIdx = static_cast<uint8_t>(group);
    
    // Check if refresh is requested or if cache is invalid
    if (refresh || !groupCache[groupIdx].valid) {
        return updateMemoryGroupCache(group, true);
    }
    
    // Check if cache is stale (older than 5 seconds)
    unsigned long currentTime = millis();
    unsigned long cacheAge = currentTime - groupCache[groupIdx].lastUpdate;
    if (cacheAge > 5000) { // 5 seconds cache validity
        return updateMemoryGroupCache(group, true);
    }
    
    // Copy data from cache
    memcpy(data, groupCache[groupIdx].values, xy_sk::DATA_GROUP_REGISTERS * sizeof(uint16_t));
    return true;
}

bool XY_SKxxx::updateMemoryGroupCache(xy_sk::MemoryGroup group, bool force) {
    uint8_t groupIdx = static_cast<uint8_t>(group);
    
    // Only update if forced or cache is invalid
    if (force || !groupCache[groupIdx].valid) {
        uint16_t startAddr = xy_sk::DataGroupManager::getGroupStartAddress(group);
        bool success = readRegisters(startAddr, xy_sk::DATA_GROUP_REGISTERS, groupCache[groupIdx].values);
        
        if (success) {
            groupCache[groupIdx].valid = true;
            groupCache[groupIdx].lastUpdate = millis();
        } else {
            groupCache[groupIdx].valid = false;
        }
        
        return success;
    }
    
    return true;
}

// Battery cutoff methods
bool XY_SKxxx::setBatteryCutoffCurrent(float current) {
  // Battery cutoff current is stored with 3 decimal places
  uint16_t value = current * 1000;
  
  waitForSilentInterval();
  
  bool success = writeRegister(REG_BTF, value);
  if (success) {
    _protection.batteryCutoffCurrent = current;
    _lastBatteryCutoffUpdate = millis();
  }
  
  return success;
}

bool XY_SKxxx::getBatteryCutoffCurrent(float &current) {
  uint16_t value;
  
  waitForSilentInterval();
  
  if (readRegister(REG_BTF, value)) {
    current = value / 1000.0f;
    _protection.batteryCutoffCurrent = current;
    _lastBatteryCutoffUpdate = millis();
    return true;
  }
  
  return false;
}
