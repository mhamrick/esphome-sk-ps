#ifndef XY_SKXXX_CD_DATA_GROUP_H
#define XY_SKXXX_CD_DATA_GROUP_H

#include <stdint.h>

namespace xy_sk {

// Constants for data group configuration
constexpr uint16_t DATA_GROUP_BASE_ADDR = 0x0050;     // M0 starting address (0050H)
constexpr uint16_t DATA_GROUP_SIZE = 0x0010;          // 16 bytes per group (14 registers used)
constexpr uint16_t DATA_GROUP_REGISTERS = 14;         // Number of registers in each group
constexpr uint16_t EXTRACT_M_REGISTER = 0x001D;       // Register to call memory groups

// Memory Group Register Offsets (from the base address of each memory group)
enum class GroupRegisterOffset : uint8_t {
    VOLTAGE_SET      = 0x00, // 0x0050: Set voltage value
    CURRENT_SET      = 0x01, // 0x0051: Set current value
    POWER_SET        = 0x02, // 0x0052: Set power value
    OVP_SET          = 0x03, // 0x0053: Over voltage protection value
    OCP_SET          = 0x04, // 0x0054: Over current protection value
    OPP_SET          = 0x05, // 0x0055: Over power protection value
    OAH_SET          = 0x06, // 0x0056: Over amp-hour protection value
    OWH_SET          = 0x07, // 0x0057: Over watt-hour protection value
    UVP_SET          = 0x08, // 0x0058: Under voltage protection value
    UCP_SET          = 0x09, // 0x0059: Under current protection value
    VOLTAGE_BACK     = 0x0A, // 0x005A: Back to preset voltage value
    CURRENT_BACK     = 0x0B, // 0x005B: Back to preset current value
    POWER_BACK       = 0x0C, // 0x005C: Back to preset power value
    PARAMETERS       = 0x0D  // 0x005D: Parameter settings (bit flags)
};

// Enum for memory groups
enum class MemoryGroup {
    M0 = 0,
    M1 = 1,
    M2 = 2,
    M3 = 3,
    M4 = 4,
    M5 = 5,
    M6 = 6,
    M7 = 7,
    M8 = 8,
    M9 = 9
};

// Structure to hold the data for a single memory group
struct MemoryGroupData {
    uint16_t values[DATA_GROUP_REGISTERS];
    bool valid;
    unsigned long lastUpdate;
};

class DataGroupManager {
public:
    /**
     * Calculate the starting address for a given memory group
     * 
     * @param group Memory group (M0-M9)
     * @return Starting register address for the memory group
     */
    static uint16_t getGroupStartAddress(MemoryGroup group);
    
    /**
     * Calculate the address for a specific register within a memory group
     * 
     * @param group Memory group (M0-M9)
     * @param offset Register offset within the group (0-13)
     * @return Register address
     */
    static uint16_t getRegisterAddress(MemoryGroup group, uint8_t offset);
    
    /**
     * Get the address for a specific register within a memory group
     * 
     * @param group Memory group (M0-M9)
     * @param regOffset Specific register offset from GroupRegisterOffset enum
     * @return Register address
     */
    static uint16_t getRegisterAddress(MemoryGroup group, GroupRegisterOffset regOffset);
    
    /**
     * Call a memory group to make it active (copy to M0)
     * This writes to the EXTRACT_M register
     * 
     * @param group Memory group to call (M1-M9, M0 is ignored)
     * @param modbusWrite Function to write to Modbus register
     * @return true if successful
     */
    template<typename WriteFunc>
    static bool callMemoryGroup(MemoryGroup group, WriteFunc modbusWrite) {
        // M0 is already active, no need to call it
        if (group == MemoryGroup::M0) {
            return true;
        }
        
        // Write memory group number to EXTRACT_M register (001DH)
        return modbusWrite(EXTRACT_M_REGISTER, static_cast<uint16_t>(group));
    }
    
    /**
     * Read all registers from a memory group
     * 
     * @param group Memory group to read from
     * @param data Array to store the read data (must be able to hold DATA_GROUP_REGISTERS values)
     * @param modbusReadMultiple Function to read multiple Modbus registers
     * @return true if successful
     */
    template<typename ReadFunc>
    static bool readMemoryGroup(MemoryGroup group, uint16_t* data, ReadFunc modbusReadMultiple) {
        uint16_t startAddr = getGroupStartAddress(group);
        return modbusReadMultiple(startAddr, DATA_GROUP_REGISTERS, data);
    }
    
    /**
     * Write all registers to a memory group
     * 
     * @param group Memory group to write to
     * @param data Array containing the data to write (must contain DATA_GROUP_REGISTERS values)
     * @param modbusWriteMultiple Function to write multiple Modbus registers
     * @return true if successful
     */
    template<typename WriteFunc>
    static bool writeMemoryGroup(MemoryGroup group, const uint16_t* data, WriteFunc modbusWriteMultiple) {
        uint16_t startAddr = getGroupStartAddress(group);
        return modbusWriteMultiple(startAddr, DATA_GROUP_REGISTERS, data);
    }
    
    /**
     * Read a specific register from a memory group
     * 
     * @param group Memory group to read from
     * @param regOffset Specific register offset from GroupRegisterOffset enum
     * @param value Reference to store the read value
     * @param modbusRead Function to read a Modbus register
     * @return true if successful
     */
    template<typename ReadFunc>
    static bool readGroupRegister(MemoryGroup group, GroupRegisterOffset regOffset, uint16_t& value, ReadFunc modbusRead) {
        uint16_t addr = getRegisterAddress(group, regOffset);
        return modbusRead(addr, value);
    }
    
    /**
     * Write to a specific register in a memory group
     * 
     * @param group Memory group to write to
     * @param regOffset Specific register offset from GroupRegisterOffset enum
     * @param value Value to write
     * @param modbusWrite Function to write to a Modbus register
     * @return true if successful
     */
    template<typename WriteFunc>
    static bool writeGroupRegister(MemoryGroup group, GroupRegisterOffset regOffset, uint16_t value, WriteFunc modbusWrite) {
        uint16_t addr = getRegisterAddress(group, regOffset);
        return modbusWrite(addr, value);
    }
};

} // namespace xy_sk

// Now add methods to the XY_SKxxx class for memory group interactions with caching
class XY_SKxxx;  // Forward declaration

// Add these method declarations to XY_SKxxx.h
// bool readMemoryGroup(xy_sk::MemoryGroup group, uint16_t* data, bool force = false);
// bool writeMemoryGroup(xy_sk::MemoryGroup group, const uint16_t* data);
// bool callMemoryGroup(xy_sk::MemoryGroup group);
// bool getCachedMemoryGroup(xy_sk::MemoryGroup group, uint16_t* data, bool refresh = false);
// bool updateMemoryGroupCache(xy_sk::MemoryGroup group, bool force = false);

#endif // XY_SKXXX_CD_DATA_GROUP_H