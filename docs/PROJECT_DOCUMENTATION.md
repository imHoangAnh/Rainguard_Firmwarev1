# RainGuard Firmware - Tài Liệu Dự Án

## 1. Overview (Tổng Quan)

### 1.1. Giới Thiệu Dự Án
**RainGuard Firmware** là hệ thống firmware IoT chạy trên ESP32-S3-WROOM-N16R8, được thiết kế để giám sát môi trường và chuyển động theo thời gian thực. Dự án tích hợp nhiều cảm biến, GPS, camera và kết nối cloud để thu thập, xử lý và truyền tải dữ liệu môi trường.

### 1.2. Thông Tin Kỹ Thuật
- **Platform**: ESP-IDF v5.x
- **Hardware**: ESP32-S3-WROOM-N16R8 (16MB Flash, 8MB PSRAM)
- **RTOS**: FreeRTOS (dual-core)
- **Ngôn ngữ**: C/C++
- **Architecture**: Component-based modular design

### 1.3. Tính Năng Chính
- ✅ Thu thập dữ liệu từ nhiều cảm biến (BME680, MPU6050, GPS NEO-6M)
- ✅ Chụp ảnh định kỳ với camera OV2640
- ✅ Kết nối WiFi và MQTT để truyền dữ liệu
- ✅ Upload ảnh lên Cloudinary qua HTTP
- ✅ Xử lý đa nhiệm với FreeRTOS (2 cores)
- ✅ Logging chi tiết qua UART console

---

## 2. Business Context (Bối Cảnh Kinh Doanh)

### 2.1. Vấn Đề Cần Giải Quyết
Trong các ứng dụng giám sát môi trường, nông nghiệp thông minh, hoặc IoT công nghiệp, cần có giải pháp:
- **Thu thập dữ liệu đa chiều**: Nhiệt độ, độ ẩm, áp suất, chất lượng không khí, gia tốc, góc nghiêng, vị trí GPS
- **Giám sát trực quan**: Hình ảnh định kỳ từ hiện trường
- **Truyền tải real-time**: Dữ liệu cần được gửi lên cloud ngay lập tức
- **Chi phí hợp lý**: Sử dụng phần cứng giá rẻ nhưng hiệu năng cao

### 2.2. Giá Trị Mang Lại
- **Giám sát toàn diện**: Kết hợp cảm biến + GPS + camera
- **Real-time monitoring**: Dữ liệu được cập nhật liên tục qua MQTT
- **Khả năng mở rộng**: Kiến trúc modular dễ dàng thêm tính năng
- **Tiết kiệm chi phí**: Sử dụng ESP32-S3 với giá thành thấp
- **Độ tin cậy cao**: FreeRTOS đảm bảo xử lý đa nhiệm ổn định

### 2.3. Use Cases (Trường Hợp Sử Dụng)
1. **Nông nghiệp thông minh**: Giám sát điều kiện môi trường cây trồng
2. **Giám sát công trình**: Theo dõi độ nghiêng, chuyển động của công trình
3. **Quan trắc môi trường**: Thu thập dữ liệu chất lượng không khí
4. **Logistics**: Theo dõi vị trí và điều kiện vận chuyển hàng hóa
5. **An ninh**: Giám sát khu vực với camera + cảm biến chuyển động

---

## 3. Functional Requirements (Yêu Cầu Chức Năng)

### 3.1. FR-01: Quản Lý Kết Nối Mạng

#### FR-01.1: Kết Nối WiFi
- **Mô tả**: Hệ thống phải kết nối WiFi tự động khi khởi động
- **Input**: SSID và Password từ Kconfig
- **Output**: Kết nối WiFi thành công, nhận IP address
- **Retry Logic**: Tối đa 10 lần thử, mỗi lần cách nhau 5 giây
- **Error Handling**: Log lỗi và dừng hệ thống nếu không kết nối được

#### FR-01.2: Kết Nối MQTT
- **Mô tả**: Kết nối đến MQTT broker sau khi có WiFi
- **Protocol**: MQTT 3.1.1 hoặc 5.0
- **Security**: Hỗ trợ SSL/TLS
- **Auto-reconnect**: Tự động kết nối lại khi mất kết nối
- **QoS**: QoS 1 (at least once delivery)

### 3.2. FR-02: Thu Thập Dữ Liệu Cảm Biến

#### FR-02.1: Cảm Biến BME680 (Môi Trường)
- **Chức năng**: Đo nhiệt độ, độ ẩm, áp suất, chất lượng không khí
- **Giao tiếp**: I2C (address 0x77)
- **Tần suất đọc**: Theo CONFIG_SENSOR_INTERVAL_MS (mặc định: 5000ms)
- **Dữ liệu thu thập**:
  - Temperature (°C): ±1°C accuracy
  - Humidity (%): ±3% accuracy
  - Pressure (hPa): ±1 hPa accuracy
  - Gas Resistance (Ω): Chất lượng không khí
  - IAQ (Index): Indoor Air Quality score (0-500)
  - IAQ Accuracy: Độ chính xác của IAQ (0-3)

#### FR-02.2: Cảm Biến MPU6050 (Chuyển Động)
- **Chức năng**: Đo gia tốc 3 trục, tốc độ góc 3 trục
- **Giao tiếp**: I2C (address 0x68)
- **Tần suất đọc**: Theo CONFIG_SENSOR_INTERVAL_MS
- **Dữ liệu thu thập**:
  - Acceleration (g): X, Y, Z axis
  - Gyroscope (°/s): X, Y, Z axis
  - Attitude (°): Pitch, Roll, Yaw (tính toán từ Kalman filter)
  - Motion Detection: Boolean (threshold 0.2g)
- **Calibration**: Tự động calibrate gyroscope khi khởi động (200 samples)

#### FR-02.3: GPS NEO-6M
- **Chức năng**: Xác định vị trí địa lý
- **Giao tiếp**: UART1 (9600 baud)
- **Protocol**: NMEA 0183
- **Tần suất đọc**: Theo CONFIG_SENSOR_INTERVAL_MS
- **Dữ liệu thu thập**:
  - Latitude (°): Vĩ độ
  - Longitude (°): Kinh độ
  - Altitude (m): Độ cao
  - Speed (km/h): Tốc độ di chuyển
  - Course (°): Hướng di chuyển
  - Satellites: Số vệ tinh kết nối
  - HDOP: Horizontal Dilution of Precision
  - Valid: GPS fix status

### 3.3. FR-03: Chụp Ảnh và Upload

#### FR-03.1: Camera OV2640
- **Chức năng**: Chụp ảnh định kỳ
- **Giao tiếp**: Parallel camera interface (8-bit DVP)
- **Tần suất chụp**: Theo CONFIG_CAMERA_INTERVAL_MS (mặc định: 60000ms)
- **Format**: JPEG
- **Resolution**: Configurable (QVGA, VGA, SVGA, XGA, SXGA, UXGA)
- **Frame Buffer**: Lưu trong PSRAM

#### FR-03.2: Upload Cloudinary
- **Chức năng**: Upload ảnh lên Cloudinary cloud storage
- **Protocol**: HTTP POST multipart/form-data
- **Security**: HTTPS
- **Retry**: 3 lần nếu thất bại
- **Response**: Lưu URL của ảnh đã upload

### 3.4. FR-04: Truyền Dữ Liệu MQTT

#### FR-04.1: Định Dạng JSON Payload
```json
{
  "device_id": "RAINGUARD_001",
  "timestamp": 1234567890,
  "bme680": {
    "temperature": 25.5,
    "humidity": 60.2,
    "pressure": 1013.25,
    "gas_resistance": 150000,
    "iaq": 50.5,
    "iaq_accuracy": 3
  },
  "mpu6050": {
    "accel": {"x": 0.05, "y": 0.02, "z": 1.0},
    "gyro": {"x": 0.5, "y": -0.3, "z": 0.1},
    "attitude": {"pitch": 2.5, "roll": -1.2, "yaw": 45.0},
    "motion_detected": false
  },
  "gps": {
    "latitude": 21.028511,
    "longitude": 105.804817,
    "altitude": 10.5,
    "speed": 0.0,
    "course": 0.0,
    "satellites": 8,
    "hdop": 1.2,
    "valid": true
  }
}
```

#### FR-04.2: MQTT Topic
- **Topic**: `rainguard/{device_id}/telemetry`
- **QoS**: 1 (at least once)
- **Retain**: false

### 3.5. FR-05: Logging và Debug

#### FR-05.1: Console Logging
- **Interface**: UART0 (115200 baud)
- **Log Levels**: ERROR, WARN, INFO, DEBUG, VERBOSE
- **Format**: `[Timestamp] [Level] [Tag] Message`
- **Components**: Mỗi component có tag riêng

---

## 4. Luồng Hoạt Động / Workflow

### 4.1. Workflow Tổng Quan

```
┌─────────────────────────────────────────────────────────────┐
│                    SYSTEM STARTUP                           │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  1. Initialize NVS Flash                                    │
│  2. Initialize Network Stack (WiFi + MQTT)                  │
│  3. Wait for WiFi Connection (max 10 retries)               │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  4. Initialize I2C Bus (SDA: GPIO1, SCL: GPIO2)             │
│  5. Scan I2C Bus for Devices                                │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  6. Initialize Sensors:                                     │
│     - BME680 (I2C 0x77) + Configure                         │
│     - MPU6050 (I2C 0x68) + Calibrate Gyro                   │
│  7. Initialize GPS (UART1, 9600 baud)                       │
│  8. Initialize Camera OV2640                                │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  9. Create FreeRTOS Tasks:                                  │
│     - Sensor Task (Core 1, Priority 5)                      │
│     - Camera Task (Core 0, Priority 5)                      │
└─────────────────────────────────────────────────────────────┘
                            │
                ┌───────────┴───────────┐
                ▼                       ▼
    ┌──────────────────┐    ┌──────────────────┐
    │  SENSOR TASK     │    │  CAMERA TASK     │
    │  (Core 1)        │    │  (Core 0)        │
    └──────────────────┘    └──────────────────┘
```

### 4.2. Sensor Task Workflow (Core 1)

```
┌─────────────────────────────────────────────────────────────┐
│                    SENSOR TASK LOOP                         │
│              (Every CONFIG_SENSOR_INTERVAL_MS)              │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  1. Read BME680 Sensor                                      │
│     - Temperature, Humidity, Pressure                       │
│     - Gas Resistance, IAQ                                   │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  2. Read MPU6050 Sensor                                     │
│     - Acceleration (X, Y, Z)                                │
│     - Gyroscope (X, Y, Z)                                   │
│     - Calculate Attitude (Pitch, Roll, Yaw)                 │
│     - Detect Motion                                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  3. Read GPS NEO-6M (Timeout: 1 second)                     │
│     - Parse NMEA sentences                                  │
│     - Extract position, speed, satellites                   │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  4. Format JSON Payload                                     │
│     - Combine all sensor data                               │
│     - Add device_id and timestamp                           │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  5. Publish to MQTT                                         │
│     - Topic: rainguard/{device_id}/telemetry                │
│     - QoS: 1                                                │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  6. Wait for Next Interval                                  │
│     - vTaskDelayUntil()                                     │
└─────────────────────────────────────────────────────────────┘
                            │
                            └──────────► (Loop back to step 1)
```

### 4.3. Camera Task Workflow (Core 0)

```
┌─────────────────────────────────────────────────────────────┐
│                    CAMERA TASK LOOP                         │
│              (Every CONFIG_CAMERA_INTERVAL_MS)              │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  1. Capture Frame from OV2640                               │
│     - Format: JPEG                                          │
│     - Buffer: PSRAM                                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  2. Upload to Cloudinary                                    │
│     - HTTP POST multipart/form-data                         │
│     - HTTPS connection                                      │
│     - Retry: 3 times if failed                              │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  3. Free Frame Buffer                                       │
│     - Release PSRAM memory                                  │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  4. Wait for Next Interval                                  │
│     - vTaskDelayUntil()                                     │
└─────────────────────────────────────────────────────────────┘
                            │
                            └──────────► (Loop back to step 1)
```

### 4.4. Network Connection Workflow

```
┌─────────────────────────────────────────────────────────────┐
│              WiFi Connection Workflow                       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  1. Initialize WiFi Driver                                  │
│     - Set mode: Station (STA)                               │
│     - Configure SSID and Password                           │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  2. Start WiFi Connection                                   │
└─────────────────────────────────────────────────────────────┘
                            │
                ┌───────────┴───────────┐
                ▼                       ▼
    ┌──────────────────┐    ┌──────────────────┐
    │   Connected      │    │   Failed         │
    └──────────────────┘    └──────────────────┘
                │                       │
                │                       ▼
                │           ┌──────────────────┐
                │           │ Retry < 10?      │
                │           └──────────────────┘
                │                   │       │
                │                   │ Yes   │ No
                │                   ▼       ▼
                │           ┌──────────┐  ┌──────┐
                │           │ Wait 5s  │  │ FAIL │
                │           └──────────┘  └──────┘
                │                   │
                │                   └──► (Loop back to step 2)
                ▼
┌─────────────────────────────────────────────────────────────┐
│  3. Get IP Address (DHCP)                                   │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  4. Initialize MQTT Client                                  │
│     - Connect to broker                                     │
│     - Set auto-reconnect                                    │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
                      ┌──────────┐
                      │  READY   │
                      └──────────┘
```

---

## 5. Non-Functional Requirements (Yêu Cầu Phi Chức Năng)

### 5.1. NFR-01: Performance (Hiệu Năng)

#### NFR-01.1: Sensor Reading Performance
- **Latency**: Thời gian đọc mỗi cảm biến < 100ms
- **Throughput**: Có thể đọc tất cả cảm biến trong < 500ms
- **CPU Usage**: Sensor task sử dụng < 30% CPU core 1

#### NFR-01.2: MQTT Publishing Performance
- **Latency**: Thời gian publish message < 200ms
- **Throughput**: Có thể publish 1 message/5s ổn định
- **Network Bandwidth**: ~500 bytes/message

#### NFR-01.3: Camera Performance
- **Capture Time**: < 1 second để chụp 1 frame
- **Upload Time**: < 5 seconds để upload 1 ảnh (tùy kích thước)
- **Memory**: Sử dụng PSRAM cho frame buffer (không ảnh hưởng RAM chính)

### 5.2. NFR-02: Reliability (Độ Tin Cậy)

#### NFR-02.1: System Uptime
- **Target**: 99% uptime trong 24h
- **Recovery**: Tự động reconnect WiFi/MQTT khi mất kết nối
- **Watchdog**: Sử dụng task watchdog để phát hiện task bị treo

#### NFR-02.2: Data Integrity
- **Sensor Data**: Validate dữ liệu trước khi publish
- **MQTT QoS**: QoS 1 đảm bảo message được gửi ít nhất 1 lần
- **Error Handling**: Log tất cả lỗi, không crash hệ thống

#### NFR-02.3: Fault Tolerance
- **Sensor Failure**: Hệ thống vẫn hoạt động nếu 1 cảm biến lỗi
- **Network Failure**: Retry logic cho WiFi và MQTT
- **Camera Failure**: Log lỗi nhưng không ảnh hưởng sensor task

### 5.3. NFR-03: Scalability (Khả Năng Mở Rộng)

#### NFR-03.1: Component Architecture
- **Modular Design**: Mỗi component độc lập, dễ thêm/bớt
- **Interface**: API rõ ràng giữa các component
- **Configuration**: Sử dụng Kconfig cho cấu hình linh hoạt

#### NFR-03.2: Resource Management
- **Memory**: Sử dụng PSRAM cho data lớn (camera frame)
- **Task Stack**: Đủ lớn để tránh overflow (sensor: 4KB, camera: 8KB)
- **Flash**: Partition table hỗ trợ OTA update

### 5.4. NFR-04: Maintainability (Khả Năng Bảo Trì)

#### NFR-04.1: Code Quality
- **Coding Standard**: Tuân thủ ESP-IDF coding style
- **Documentation**: Mỗi function có Doxygen comment
- **Naming**: Tên biến, hàm rõ ràng, có ý nghĩa

#### NFR-04.2: Logging
- **Log Levels**: Sử dụng đúng level (ERROR, WARN, INFO, DEBUG)
- **Log Tags**: Mỗi component có tag riêng
- **Debug Mode**: CONFIG_LOG_DEFAULT_LEVEL_DEBUG để debug

#### NFR-04.3: Testing
- **Unit Test**: Test từng component độc lập
- **Integration Test**: Test tương tác giữa các component
- **Field Test**: Test trong môi trường thực tế

### 5.5. NFR-05: Security (Bảo Mật)

#### NFR-05.1: Network Security
- **WiFi**: WPA2/WPA3 encryption
- **MQTT**: Hỗ trợ SSL/TLS
- **HTTP**: HTTPS cho Cloudinary upload

#### NFR-05.2: Credential Management
- **Storage**: Lưu credentials trong NVS (encrypted)
- **Configuration**: Không hardcode credentials trong code
- **Access Control**: Chỉ authorized devices mới publish được

### 5.6. NFR-06: Usability (Khả Năng Sử Dụng)

#### NFR-06.1: Configuration
- **Kconfig**: GUI để cấu hình (idf.py menuconfig)
- **Defaults**: Giá trị mặc định hợp lý
- **Validation**: Kiểm tra giá trị cấu hình hợp lệ

#### NFR-06.2: Monitoring
- **Console Log**: Real-time log qua UART
- **MQTT**: Có thể subscribe để xem dữ liệu
- **Status LED**: (Nếu có) Hiển thị trạng thái hệ thống

### 5.7. NFR-07: Portability (Tính Di Động)

#### NFR-07.1: Hardware Compatibility
- **ESP32-S3**: Tối ưu cho ESP32-S3-WROOM-N16R8
- **Pin Configuration**: Dễ dàng thay đổi trong pin_config.h
- **Sensor Support**: Có thể thay thế sensor tương tự

#### NFR-07.2: Software Compatibility
- **ESP-IDF**: Tương thích v5.x
- **Dependencies**: Sử dụng managed components từ ESP Component Registry
- **Build System**: CMake standard

### 5.8. NFR-08: Resource Constraints (Ràng Buộc Tài Nguyên)

#### NFR-08.1: Memory
- **RAM**: < 200KB cho application
- **PSRAM**: Sử dụng cho camera frame buffer
- **Flash**: < 2MB cho firmware

#### NFR-08.2: Power Consumption
- **Active Mode**: ~200mA @ 3.3V
- **WiFi TX**: Peak ~400mA
- **Camera**: Peak ~300mA
- **Total**: < 500mA average

#### NFR-08.3: Timing
- **Boot Time**: < 5 seconds từ power-on đến ready
- **Task Period**: Sensor 5s, Camera 60s (configurable)
- **Real-time**: Không yêu cầu hard real-time

---

## 6. Kiến Trúc Hệ Thống

### 6.1. Component Architecture

```
rainguard/
├── main/
│   └── main.c                  # Entry point, task orchestration
├── components/
│   ├── app_config/             # Pin configuration
│   │   └── include/pin_config.h
│   ├── app_network/            # WiFi + MQTT + HTTP
│   │   ├── include/app_network.h
│   │   └── app_network.c
│   ├── system_i2c/             # I2C bus driver
│   │   ├── include/system_i2c.h
│   │   └── system_i2c.c
│   ├── sensor_bme680/          # BME680 environment sensor
│   │   ├── include/sensor_bme680.h
│   │   ├── sensor_bme680.c
│   │   └── driver/bme68x.*     # Bosch driver
│   ├── sensor_mpu6050/         # MPU6050 motion sensor
│   │   ├── include/sensor_mpu6050.h
│   │   ├── sensor_mpu6050.c
│   │   └── driver/mpu6050.*
│   ├── gps_neo6m/              # GPS module
│   │   ├── include/gps_neo6m.h
│   │   └── gps_neo6m.c
│   └── cam_config/             # OV2640 camera
│       ├── include/cam_config.h
│       ├── cam_config.c
│       └── driver/ov2640.*
└── managed_components/         # External dependencies
```
---

## 7. Hướng Dẫn Sử Dụng

### 7.1. Yêu Cầu Hệ Thống
- ESP-IDF v5.5.2
- Python 3.8+
- USB driver cho ESP32-S3

### 7.2. Cấu Hình Dự Án

```bash
# 1. Clone repository
git clone <repository_url>
cd Rainguard_Firmware

# 2. Cấu hình menuconfig
idf.py menuconfig

# Cấu hình các mục sau:
# - WiFi SSID và Password
# - MQTT Broker URL, Username, Password
# - Cloudinary credentials
# - Device ID
# - Sensor/Camera intervals
```

### 7.3. Build và Flash

```bash
# Build project
idf.py build

# Flash to ESP32-S3
idf.py -p COM8 flash

# Monitor console output
idf.py -p COM8 monitor

# Hoặc flash + monitor cùng lúc
idf.py -p COM8 flash monitor
```

### 7.4. Troubleshooting

#### Lỗi I2C: Sensor không phát hiện
- Kiểm tra kết nối SDA/SCL
- Chạy `system_i2c_scan()` để xem địa chỉ I2C
- Kiểm tra pull-up resistor (4.7kΩ)

#### Lỗi WiFi: Không kết nối được
- Kiểm tra SSID/Password trong menuconfig
- Kiểm tra router có bật không
- Xem log để biết lỗi cụ thể

#### Lỗi Camera: Không chụp được ảnh
- Kiểm tra kết nối camera FPC
- Kiểm tra PSRAM đã enable chưa
- Giảm resolution nếu thiếu memory

---

## 8. Tài Liệu Tham Khảo

### 8.1. Hardware Datasheets
- [ESP32-S3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [BME680 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [NEO-6M GPS Datasheet](https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf)
- [OV2640 Datasheet](https://www.uctronics.com/download/cam_module/OV2640DS.pdf)

### 8.2. Software Documentation
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [MQTT Protocol](https://mqtt.org/mqtt-specification/)
- [Cloudinary API](https://cloudinary.com/documentation)

### 8.3. Related Projects
- [ESP32-CAM Examples](https://github.com/espressif/esp32-camera)
- [BME680 BSEC Library](https://github.com/BoschSensortec/BSEC-Arduino-library)

---

## 9. Changelog

### Version 1.0.0 (Current)
- ✅ Initial release
- ✅ BME680, MPU6050, GPS, Camera integration
- ✅ WiFi + MQTT connectivity
- ✅ Cloudinary image upload
- ✅ FreeRTOS dual-core task management

### Planned Features
- 🔄 OTA firmware update
- 🔄 SD card logging
- 🔄 Deep sleep mode for power saving
- 🔄 Web server for local configuration
- 🔄 BLE provisioning

---

## 10. License & Contact

### License
This project is proprietary software. All rights reserved.

### Contact
- **Developer**: [Tran Hoang Anh]
- **Email**: [trhoanganh2503@gmail.com]
- **Project**: RainGuard Firmware

---

**Document Version**: 1.0  
**Last Updated**: 2026-01-12  
**Status**: Production Ready
