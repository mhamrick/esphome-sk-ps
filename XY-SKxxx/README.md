# XY-SKxxx Modbus RTU Library

A lightweight library for controlling XY-SK series digital power supplies using Modbus RTU protocol over TTL.

## Overview

This library provides a complete interface to the XY-SK series digital power supplies (DPS), including the XY-SK120 model. It allows you to:

- Control output voltage and current settings
- Monitor actual output voltage, current, and power
- Turn the output on/off
- Set constant voltage (CV) or constant current (CC) modes
- Configure protection settings (OVP, OCP, OPP, OTP, etc.)
- Read and control device status (keypad lock, brightness, etc.)
- Direct register access for debugging

## Installation

### PlatformIO

1. Add the library to your `platformio.ini` file:

```ini
lib_deps =
    https://github.com/csvke/XY-SK120-Modbus-RTU-TTL.git
    4-20ma/ModbusMaster@^2.0.1
```

2. Or copy this folder (`XY-SKxxx`) to your project's `lib` directory.

### Arduino IDE

1. Download this repository as ZIP
2. In Arduino IDE, go to Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file
4. Make sure to also install the ModbusMaster library via Library Manager

## Basic Usage

```cpp
#include <Arduino.h>
#include "XY-SKxxx.h"

// Define pins for TTL communication
#define RX_PIN D7  // Connect to TX of the power supply
#define TX_PIN D6  // Connect to RX of the power supply
#define SLAVE_ID 1 // Default Modbus slave ID is usually 1

// Create power supply instance
XY_SKxxx powerSupply(RX_PIN, TX_PIN, SLAVE_ID);

void setup() {
  Serial.begin(115200);
  
  // Initialize the power supply with baud rate
  powerSupply.begin(115200);
  
  // Test if connection works
  if (powerSupply.testConnection()) {
    Serial.println("Connected to power supply!");
    
    // Set voltage to 5V and current to 1A
    powerSupply.setVoltageAndCurrent(5.0, 1.0);
    
    // Turn output on
    powerSupply.turnOutputOn();
  } else {
    Serial.println("Failed to connect to power supply");
  }
}

void loop() {
  // Read current output values
  float voltage, current, power;
  bool isOn;
  
  if (powerSupply.getOutputStatus(voltage, current, power, isOn)) {
    Serial.print("V: "); Serial.print(voltage, 2);
    Serial.print(" A: "); Serial.print(current, 3);
    Serial.print(" W: "); Serial.print(power, 3);
    Serial.print(" Output: "); Serial.println(isOn ? "ON" : "OFF");
  }
  
  delay(1000);
}
```

## Advanced Usage with Configuration Storage

For ESP32 platforms, the library includes a configuration manager that can store settings in non-volatile storage:

```cpp
#include <Arduino.h>
#include "XY-SKxxx.h"
#include "XY-SKxxx_Config.h"

// Global configuration instance
XYModbusConfig config;
XY_SKxxx* powerSupply = nullptr;

void setup() {
  Serial.begin(115200);
  
  // Initialize configuration manager
  XYConfigManager::begin();
  
  // Load saved configuration or use defaults
  config = XYConfigManager::loadConfig();
  
  // Create power supply instance with loaded configuration
  powerSupply = new XY_SKxxx(config.rxPin, config.txPin, config.slaveId);
  powerSupply->begin(config.baudRate);
  
  // Example of saving new settings
  config.baudRate = 115200;
  XYConfigManager::saveConfig(config);
}
```

## Key Features

### Power Control

```cpp
// Set output parameters
powerSupply.setVoltage(12.0);               // Set voltage to 12V
powerSupply.setCurrent(0.5);                // Set current to 0.5A
powerSupply.setVoltageAndCurrent(12.0, 0.5); // Set both at once

// Control output state
powerSupply.turnOutputOn();                 // Turn output on
powerSupply.turnOutputOff();                // Turn output off
```

### Measurements

```cpp
// Read output parameters individually
float voltage = powerSupply.getOutputVoltage(true); // true = refresh cache
float current = powerSupply.getOutputCurrent(true);
float power = powerSupply.getOutputPower(true);

// Read multiple values at once
float v, c, p;
bool isOn;
powerSupply.getOutputStatus(v, c, p, isOn);

// Read input voltage
float inputV = powerSupply.getInputVoltage();

// Read energy meters
uint32_t ampHours = powerSupply.getAmpHours();
uint32_t wattHours = powerSupply.getWattHours();
```

### Protection Settings

```cpp
// Set protection thresholds
powerSupply.setOverVoltageProtection(15.0);    // OVP: 15V
powerSupply.setOverCurrentProtection(2.0);     // OCP: 2A
powerSupply.setOverPowerProtection(20.0);      // OPP: 20W
powerSupply.setOverTemperatureProtection(70.0); // OTP: 70°C
```

### Operating Modes

```cpp
// Set constant voltage mode
powerSupply.setConstantVoltage(5.0);  // CV: 5V

// Set constant current mode
powerSupply.setConstantCurrent(1.0);  // CC: 1A

// Check current operating mode
if (powerSupply.isInConstantCurrentMode()) {
  // In CC mode
} else if (powerSupply.isInConstantVoltageMode()) {
  // In CV mode
}
```

### Device Settings

```cpp
// Lock/unlock front panel keys
powerSupply.setKeyLock(true);  // Lock
powerSupply.setKeyLock(false); // Unlock

// Set backlight brightness (0-5)
powerSupply.setBacklightBrightness(3);

// Set sleep timeout in minutes
powerSupply.setSleepTimeout(5);
```

### Memory Group Access

The library supports accessing memory groups (M0-M9) for storing and recalling device settings:

```cpp
// Call a memory group (activate it)
powerSupply.callMemoryGroup(xy_sk::MemoryGroup::M1);

// Read from a memory group
uint16_t data[xy_sk::DATA_GROUP_REGISTERS];
powerSupply.readMemoryGroup(xy_sk::MemoryGroup::M2, data);

// Write to a memory group
powerSupply.writeMemoryGroup(xy_sk::MemoryGroup::M3, data);
```

### Debug Register Access

For advanced users and debugging purposes, the library provides direct register access methods:

```cpp
// Read registers directly
uint16_t values[10];
powerSupply.debugReadRegisters(0x0000, 10, values);  // Read 10 registers starting at 0x0000

// Write to a register directly
powerSupply.debugWriteRegister(0x0000, 1250);  // Write 1250 (12.50V) to voltage register

// Write to multiple registers
uint16_t writeValues[2] = {1250, 2000};  // 12.50V, 2.000A
powerSupply.debugWriteRegisters(0x0000, 2, writeValues);  // Write to voltage and current registers
```

## Serial Monitor Interface

When used with the project's serial monitor interface, you can use these debug commands:

- `read addr count` - Read 'count' registers starting at address 'addr'
- `write addr value` - Write 'value' to register at address 'addr'
- `writes addr v1 v2 ...` - Write multiple values to consecutive registers

Examples:
- `read 0x0000 1` - Read the voltage setting register
- `write 0x0000 1250` - Set voltage to 12.50V (1250/100)
- `read 0x0002 3` - Read output voltage, current, and power

## Cache System

The library implements a caching system to reduce Modbus communication overhead:

```cpp
// Force refresh of all status values
powerSupply.updateAllStatus(true);

// Get cached values without communication
float v = powerSupply.getOutputVoltage(false); // false = use cache
```

## Hardware Configuration

The library has been tested with the XY-SK120 power supply connected to a Seeed Studio XIAO ESP32S3 with the following connections:

- Power supply GND → XIAO ESP32S3 GND
- Power supply TX → XIAO ESP32S3 D7 (RX)
- Power supply RX → XIAO ESP32S3 D6 (TX)

## Troubleshooting

- **No communication**: Check wiring, baud rate, and slave ID
- **Erratic behavior**: Ensure proper timing between commands (the library handles this internally with silent intervals)
- **Protection tripping**: Verify your protection settings match your use case

## License

This library is licensed under the MIT License.
