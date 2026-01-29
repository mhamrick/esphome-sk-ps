# esphome-sk-ps

ESPHome component to monitor and control XY-SK series digital power supplies via Modbus RTU over UART-TTL.

## Acknowledgments

This project is based on [esphome-dps](https://github.com/syssi/esphome-dps) by [@syssi](https://github.com/syssi). Thank you for the excellent foundation that made this adaptation possible.

## Supported Devices

* XY-SK60 (0-60V)
* XY-SK120 (0-120V)

## Requirements

* [ESPHome 2024.6.0 or higher](https://github.com/esphome/esphome/releases)
* ESP32 or ESP8266 board (D1 Mini recommended)

## Wiring

```
┌──────────────┐                ┌─────────────┐
│              │<----- RX ----->│             │
│  XY-SK60/120 │<----- TX ----->│ ESP32/D1    │
│              │<----- GND ---->│ Mini        │<-- 3.3V
│              │                │             │<-- GND
└──────────────┘                └─────────────┘
```

**Connector**: 4 Pin GH Molex Pico 1.25mm

| Power Supply Pin | ESP Pin |
|------------------|---------|
| GND | GND |
| TXD | RX pin (D4/GPIO2 in example) |
| RXD | TX pin (D3/GPIO0 in example) |
| VCC | **DO NOT CONNECT** |

**Warning**: Do not connect the ESP to the VCC pin. This will destroy the voltage regulator.

## Installation

### Using External Components

```yaml
external_components:
  - source: github://your-username/esphome-sk-ps@main
```

### Manual Installation

```bash
# Install esphome
pip3 install esphome

# Clone this repository
git clone https://github.com/your-username/esphome-sk-ps.git
cd esphome-sk-ps

# Create secrets.yaml
cat > secrets.yaml <<EOF
wifi_ssid: MY_WIFI_SSID
wifi_password: MY_WIFI_PASSWORD
EOF

# Flash to device
esphome run sk60.yaml
```

## Features

### Sensors (Read-Only)
| Sensor | Description |
|--------|-------------|
| Output Voltage | Current output voltage (V) |
| Output Current | Current output current (A) |
| Output Power | Current output power (W) |
| Input Voltage | Power supply input voltage (V) |
| Internal Temperature | Device temperature (C) |
| Voltage Setting | Configured voltage setpoint |
| Current Setting | Configured current setpoint |
| Backlight Brightness | Display brightness level |
| Firmware Version | Device firmware version |

### Binary Sensors
| Sensor | Description |
|--------|-------------|
| Output | Output state (ON/OFF) |
| Key Lock | Front panel lock state |
| Constant Current Mode | CC mode indicator |

### Controls
| Control | Description |
|---------|-------------|
| Output Switch | Toggle power output ON/OFF |
| Key Lock Switch | Lock/unlock front panel |
| Voltage Setting | Set output voltage (0-60V) |
| Current Setting | Set output current (0-20A) |

### Text Sensors
| Sensor | Description |
|--------|-------------|
| Device Model | Auto-detected model name |
| Protection Status | Current protection state |

## Example Configuration

See `sk60.yaml` for a complete working example.

```yaml
substitutions:
  name: sk60
  tx_pin: D3
  rx_pin: D4

uart:
  baud_rate: 115200
  tx_pin: ${tx_pin}
  rx_pin: ${rx_pin}

modbus:
  send_wait_time: 0ms

xysk:
  id: xysk0
  modbus_id: modbus0
  update_interval: 10s

sensor:
  - platform: xysk
    xysk_id: xysk0
    output_voltage:
      name: "${name} output voltage"
    output_current:
      name: "${name} output current"
    output_power:
      name: "${name} output power"

switch:
  - platform: xysk
    xysk_id: xysk0
    output:
      name: "${name} output"

number:
  - platform: xysk
    xysk_id: xysk0
    voltage_setting:
      name: "${name} voltage setting"
      max_value: 60
    current_setting:
      name: "${name} current setting"
      max_value: 20
```

## Debugging

Enable UART debug output if the component doesn't work:

```yaml
logger:
  level: DEBUG

uart:
  baud_rate: 115200
  tx_pin: ${tx_pin}
  rx_pin: ${rx_pin}
  debug:
    direction: BOTH
```

## Project Structure

```
esphome-sk-ps/
├── sk60.yaml              # Example configuration
├── docs/                  # Hardware documentation
├── XY-SKxxx/              # Low-level Modbus library
└── components/
    ├── xysk/              # Main ESPHome component
    └── lazy_limiter/      # Power demand management (optional)
```

## License

Apache 2.0

## References

* [XY-SK Power Supply User Guide](docs/SK60_SK120_Power_Supply_User_Guide.pdf)
* [Original esphome-dps project](https://github.com/syssi/esphome-dps)
