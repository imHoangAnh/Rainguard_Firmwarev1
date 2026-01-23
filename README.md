# 🌧️ TrainGuard Firmware

<div align="center">

![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.2-blue?logo=espressif)
![License](https://img.shields.io/badge/License-MIT-green)
![Platform](https://img.shields.io/badge/Platform-ESP32--S3-orange?logo=espressif)
![Status](https://img.shields.io/badge/Status-Active-brightgreen)

**An IoT Environmental Monitoring System for Real-time Weather & Motion Tracking**

[Features](#-features) • [Hardware](#-hardware-requirements) • [Installation](#-installation) • [Configuration](#️-configuration) • [API](#-mqtt-json-api) • [Documentation](#-documentation)

</div>

---

## 📖 Overview

**TrainGuard** is an ESP32-S3 based firmware that collects environmental data from multiple sensors and transmits it via MQTT in real-time. Designed for weather monitoring stations, and environmental tracking applications.

### Key Capabilities
- 🌡️ **Environmental Monitoring** - Temperature, humidity, pressure, and air quality
- 📍 **GPS Tracking** - Real-time location with smoothing algorithm
- 📐 **Motion Detection** - 6-axis IMU for orientation and vibration analysis
- 📷 **Image Capture** - Periodic snapshots uploaded to Cloudinary
- 📡 **MQTT Communication** - JSON-formatted sensor data publishing

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| **Multi-Sensor Fusion** | BME680 (environment) + MPU6050 (motion) + GPS NEO-7M |
| **Real-time Data** | Sensor data published every 10 seconds via MQTT |
| **Cloud Image Upload** | Camera captures every 60 seconds, uploads to Cloudinary |
| **GPS Smoothing** | Exponential moving average filter for stable positioning |
| **Timezone Support** | Automatic UTC to Vietnam (UTC+7) conversion |
| **IAQ Calculation** | Indoor Air Quality index from gas resistance |
| **Docker MQTT Broker** | Included Mosquitto configuration for quick setup |

---

## 🔧 Hardware Requirements

### Main Board
- **ESP32-S3-WROOM N16R8** (16MB Flash, 8MB PSRAM)
- Recommended: Freenove ESP32-S3-WROOM CAM Board

### Sensors

| Sensor | Interface | Function |
|--------|-----------|----------|
| **BME680** | I2C (0x77) | Temperature, Humidity, Pressure, Gas/IAQ |
| **MPU6050** | I2C (0x68) | Accelerometer, Gyroscope, Attitude |
| **GPS NEO-7M** | UART (9600 baud) | Location, Speed, Time |
| **OV2640** | DVP (8-bit) | Camera (JPEG) |

### Wiring Diagram

```
ESP32-S3 Pin Connections
========================

I2C Bus (BME680 + MPU6050):
├── SDA  → GPIO 1
└── SCL  → GPIO 2

GPS NEO-7M (UART1):
├── TX   → GPIO 41
└── RX   → GPIO 42

Camera OV2640:
├── XCLK  → GPIO 15
├── SIOD  → GPIO 4 (SCCB Data)
├── SIOC  → GPIO 5 (SCCB Clock)
├── VSYNC → GPIO 6
├── HREF  → GPIO 7
├── PCLK  → GPIO 13
├── D0-D7 → GPIO 11,9,8,10,12,18,17,16
├── RESET → Not Connected
└── PWDN  → Not Connected
```

---

## 📦 Installation

### Prerequisites

- [ESP-IDF v5.5.2](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/)
- Python 3.8+
- Git

### Clone Repository

```bash
git clone https://github.com/imHoangAnh/TrainGuard_Firmware.git
cd TrainGuard_Firmware
```

### Build & Flash

```bash
# Set target to ESP32-S3
idf.py set-target esp32s3

# Configure (optional - defaults are pre-configured)
idf.py menuconfig

# Build
idf.py build

# Flash to device
idf.py -p COM8 flash

# Monitor output
idf.py -p COM8 monitor
```
## ⚙️ Configuration

### Network Settings

Edit `components/app_config/network_config.h`:

```c
// Device Configuration
#define DEVICE_ID "01"

// WiFi Configuration
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"

// MQTT Configuration
#define MQTT_BROKER_URI "mqtt://192.168.1.100:1883"
#define MQTT_TOPIC_PREFIX "trainguard/data"

// Cloudinary Configuration
#define CLOUDINARY_UPLOAD_URL "https://api.cloudinary.com/v1_1/yourcloud/image/upload"
#define CLOUDINARY_UPLOAD_PRESET "YourPreset"

// Intervals (milliseconds)
#define SENSOR_INTERVAL_MS 10000   // 10 seconds
#define CAMERA_INTERVAL_MS 60000   // 60 seconds
```

### Pin Configuration

Edit `components/app_config/pin_config.h` if using different GPIO pins.

---

## 📡 MQTT JSON API

### Topic Structure

```
trainguard/data/{device_id}
```

Example: `trainguard/data/01`

### Payload Schema

```json
{
  "device_id": "01",
  "timestamp": 1705865896000,
  "bme680": {
    "temperature": 28.50,
    "humidity": 65.30,
    "pressure": 1013.25,
    "gas_resistance": 52340,
    "iaq": 45.2,
    "iaq_accuracy": 3,
    "gas_valid": true,
    "heat_stable": true
  },
  "mpu6050": {
    "accel": { "x": 0.012, "y": -0.023, "z": 0.998 },
    "gyro": { "x": 0.15, "y": -0.08, "z": 0.02 },
    "attitude": { "pitch": 1.25, "roll": -0.85, "yaw": 45.30 },
    "vibration": 0.5
  },
  "gps": {
    "latitude": 10.762622,
    "longitude": 106.660172,
    "altitude": 15.5,
    "speed": 0.00,
    "date": "2026-01-21",
    "time": "22:21:16.000",
    "valid": true
  }
}
```

### Field Descriptions

| Field | Unit | Description |
|-------|------|-------------|
| `timestamp` | ms | System uptime in milliseconds |
| `temperature` | °C | Ambient temperature |
| `humidity` | %RH | Relative humidity |
| `pressure` | hPa | Atmospheric pressure |
| `gas_resistance` | Ω | Raw gas sensor resistance |
| `iaq` | 0-500 | Indoor Air Quality index |
| `iaq_accuracy` | 0-3 | IAQ calibration level |
| `accel.x/y/z` | g | Acceleration (gravity units) |
| `gyro.x/y/z` | °/s | Angular velocity |
| `pitch/roll/yaw` | ° | Device orientation |
| `vibration` | g | Vibration magnitude |
| `latitude/longitude` | ° | GPS coordinates |
| `altitude` | m | Altitude above sea level |
| `speed` | km/h | Ground speed |

---

## 🐳 MQTT Broker Setup

A Docker Compose file is included for quick Mosquitto setup:

```bash
# Start MQTT broker
docker-compose up -d

# View logs
docker-compose logs -f mosquitto

# Stop broker
docker-compose down
```

The broker will be available at:
- **MQTT**: `mqtt://localhost:1883`
- **WebSocket**: `ws://localhost:9001`

---

## 📁 Project Structure

```
TrainGuard_Firmware/
├── main/
│   └── main.c                  # Application entry point
├── components/
│   ├── app_config/
│   │   ├── network_config.h    # WiFi, MQTT, Cloudinary settings
│   │   └── pin_config.h        # GPIO pin definitions
│   ├── app_network/
│   │   └── app_network.c       # WiFi, MQTT, HTTP client
│   ├── sensor_bme680/          # BME680 driver (I2C)
│   ├── sensor_mpu6050/         # MPU6050 driver (I2C)
│   ├── gps_neo7m/              # GPS NEO-7M driver (UART)
│   ├── cam_config/             # OV2640 camera driver
│   └── system_i2c/             # Shared I2C bus
├── mosquitto/                   # MQTT broker config
├── docker-compose.yml           # Docker Mosquitto setup
├── partitions.csv               # Custom partition table
├── sdkconfig.defaults           # ESP-IDF default config
└── CMakeLists.txt              # Build configuration
```

## 🔍 Debugging

### Serial Monitor

```bash
idf.py -p COM8 monitor
```

### Sample Output

```
I (1234) main: TrainGuard starting...
I (2345) net: MQTT connected
I (3456) main: MQTT published successfully
I (4567) main: Captured: 640x480, 25340 bytes
I (4890) main: Uploaded image to Cloudinary successfully
```

### MQTT Subscriber (Testing)

```bash
# Using mosquitto_sub
mosquitto_sub -h localhost -t "trainguard/data/#" -v
```

---

## ⚡ Performance

| Metric | Value |
|--------|-------|
| Flash Usage | ~1.2 MB |
| RAM Usage | ~180 KB |
| PSRAM Usage | ~2 MB (camera buffer) |
| Sensor Read Cycle | ~50 ms |
| MQTT Publish Latency | ~100 ms |
| Image Upload Time | ~3-5 seconds |

---

## 🛠️ Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| WiFi not connecting | Check SSID/password in `network_config.h` |
| MQTT connection failed | Verify broker IP and port |
| GPS no fix | Ensure antenna has clear sky view |
| Camera black image | Check camera ribbon cable connection |
| BME680 invalid readings | Wait 5+ minutes for gas sensor warmup |

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👨 Author

**Hoang Anh**

---