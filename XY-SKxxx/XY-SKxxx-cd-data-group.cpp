#include "XY-SKxxx-cd-data-group.h"

namespace xy_sk {

uint16_t DataGroupManager::getGroupStartAddress(MemoryGroup group) {
    // Calculate the starting address based on the formula: 0050H + group_number * 0010H
    return DATA_GROUP_BASE_ADDR + (static_cast<uint16_t>(group) * DATA_GROUP_SIZE);
}

uint16_t DataGroupManager::getRegisterAddress(MemoryGroup group, uint8_t offset) {
    // Ensure offset is within valid range (0-13)
    if (offset >= DATA_GROUP_REGISTERS) {
        // Invalid offset, return base address as fallback
        return getGroupStartAddress(group);
    }
    
    // Calculate register address: base address + offset
    return getGroupStartAddress(group) + offset;
}

uint16_t DataGroupManager::getRegisterAddress(MemoryGroup group, GroupRegisterOffset regOffset) {
    // Convert the enum to its underlying type (uint8_t)
    return getRegisterAddress(group, static_cast<uint8_t>(regOffset));
}

} // namespace xy_sk