# u-blox NEO-7M GPS Module Integration Guide

## Tổng quan

Tài liệu này mô tả chi tiết việc tích hợp module GNSS u-blox NEO-7M vào dự án RainGuard Firmware sử dụng ESP-IDF 5.5.2.

### Thông số kỹ thuật NEO-7M

| Thông số | Giá trị |
|----------|---------|
| Hệ thống GNSS | GPS + GLONASS + QZSS + SBAS |
| Số kênh | 56 |
| Độ nhạy (tracking) | -161 dBm |
| Độ nhạy (acquisition) | -147 dBm |
| Độ chính xác vị trí | 2.5m CEP |
| Tốc độ cập nhật | 1-10 Hz |
| Điện áp hoạt động | 2.7V - 3.6V |
| Dòng tiêu thụ | 40mA (continuous), 11mA (power save) |
| Giao tiếp | UART, I2C (DDC), SPI, USB |
| Baud rate mặc định | 9600, 8N1 |

### Sự khác biệt so với NEO-6M

| Tính năng | NEO-6M | NEO-7M |
|-----------|--------|--------|
| Số kênh | 50 | 56 |
| GLONASS | Không | Có |
| AssistNow Autonomous | Có | Có (cải tiến) |
| Độ nhạy | -160 dBm | -161 dBm |
| Power save | Có | Có (cải tiến) |
| TTFF (Cold start) | 27s | 26s |

---

## Cấu trúc Component

```
components/gps_neo7m/
├── CMakeLists.txt      # Build configuration
├── gps_neo7m.h         # Public API header
└── gps_neo7m.c         # Implementation
```

---

## Sơ đồ kết nối phần cứng

### Kết nối cơ bản

```
NEO-7M Module          ESP32-S3
┌────────────┐         ┌─────────────┐
│    VCC     │────────►│   3.3V      │
│    GND     │────────►│   GND       │
│    TX      │────────►│   GPIO42    │ (RX)
│    RX      │◄────────│   GPIO41    │ (TX)
└────────────┘         └─────────────┘
```

### Lưu ý quan trọng

1. **Nguồn điện**: NEO-7M yêu cầu 2.7V-3.6V. Sử dụng nguồn 3.3V ổn định.
2. **Cross-connect**: TX của GPS nối với RX của ESP32 và ngược lại.
3. **Antenna**: Sử dụng ăng-ten active hoặc passive với ground plane tốt.
4. **JTAG**: GPIO41/42 là chân JTAG, debug qua JTAG sẽ không hoạt động khi GPS kết nối.

---

## API Reference

### Khởi tạo

```c
#include "gps_neo7m.h"

// Khởi tạo với cấu hình mặc định
bool gps_neo7m_init(void);

// Khởi tạo với cấu hình tùy chỉnh
gps_config_t config;
gps_neo7m_get_default_config(&config);
config.update_rate_hz = 5;          // 5Hz update rate
config.nav_mode = GPS_NAV_MODE_AUTOMOTIVE;
config.enable_glonass = true;
config.assistnow_autonomous = true;
bool gps_neo7m_init_with_config(&config);
```

### Đọc dữ liệu GPS

```c
gps_data_t gps_data;

// Đọc với timeout 1000ms
if (gps_neo7m_read(&gps_data, 1000)) {
    if (gps_data.valid) {
        printf("Vị trí: %.6f, %.6f\n", gps_data.latitude, gps_data.longitude);
        printf("Độ cao: %.1f m\n", gps_data.altitude);
        printf("Tốc độ: %.1f km/h\n", gps_data.speed_kmh);
        printf("Vệ tinh: %d\n", gps_data.satellites_used);
        printf("HDOP: %.1f\n", gps_data.dop.hdop);
    }
}
```

### Cấu hình Runtime

```c
// Đặt tốc độ cập nhật (1-10 Hz)
gps_neo7m_set_update_rate(5);

// Đặt chế độ navigation
gps_neo7m_set_nav_mode(GPS_NAV_MODE_AUTOMOTIVE);

// Bật/tắt GLONASS
gps_neo7m_set_glonass(true);

// Bật AssistNow Autonomous
gps_neo7m_set_assistnow_autonomous(true);

// Lưu cấu hình vào flash (chỉ NEO-7N)
gps_neo7m_save_config();
```

### Điều khiển nguồn

```c
// Chế độ full power (mặc định)
gps_neo7m_set_power_mode(GPS_POWER_FULL);

// Chế độ tiết kiệm điện
gps_neo7m_set_power_mode(GPS_POWER_SAVE);

// Chế độ backup (RTC only)
gps_neo7m_set_power_mode(GPS_POWER_BACKUP);
```

### Restart

```c
// Hot start - giữ tất cả dữ liệu, khởi động nhanh
gps_neo7m_hot_start();

// Warm start - xóa ephemeris, TTFF ~30s
gps_neo7m_warm_start();

// Cold start - xóa tất cả, TTFF ~26s
gps_neo7m_cold_start();

// Hardware reset
gps_neo7m_reset();
```

---

## Cấu trúc dữ liệu

### gps_data_t

```c
typedef struct {
    /* Vị trí */
    double latitude;            // Vĩ độ (độ), dương = Bắc
    double longitude;           // Kinh độ (độ), dương = Đông
    float altitude;             // Độ cao so với mực nước biển (m)
    float geoid_separation;     // Độ lệch geoid (m)

    /* Vận tốc */
    float speed_kmh;            // Tốc độ (km/h)
    float speed_knots;          // Tốc độ (knots)
    float course;               // Hướng di chuyển (độ, true north)
    float course_magnetic;      // Hướng di chuyển (độ, magnetic north)

    /* Thời gian */
    gps_time_t utc_time;        // Thời gian UTC
    gps_date_t utc_date;        // Ngày UTC
    uint32_t timestamp_ms;      // Timestamp hệ thống

    /* Trạng thái fix */
    bool valid;                 // True nếu có fix hợp lệ
    gps_fix_type_t fix_type;    // Loại fix (2D/3D)
    gps_fix_quality_t quality;  // Chất lượng fix

    /* Vệ tinh */
    uint8_t satellites_used;    // Số vệ tinh được sử dụng
    uint8_t satellites_view;    // Số vệ tinh trong tầm nhìn
    gps_satellite_t satellites[GPS_MAX_SATELLITES];

    /* Độ chính xác */
    gps_dop_t dop;              // DOP values (PDOP, HDOP, VDOP)
    float accuracy_h;           // Độ chính xác ngang (m)
    float accuracy_v;           // Độ chính xác dọc (m)
} gps_data_t;
```

### Navigation Modes

| Mode | Ứng dụng | Max Velocity | Max Altitude |
|------|----------|--------------|--------------|
| `GPS_NAV_MODE_PORTABLE` | Đa dụng | 310 m/s | 12000 m |
| `GPS_NAV_MODE_STATIONARY` | Cố định, timing | 0 m/s | 9000 m |
| `GPS_NAV_MODE_PEDESTRIAN` | Đi bộ | 30 m/s | 9000 m |
| `GPS_NAV_MODE_AUTOMOTIVE` | Xe hơi | 100 m/s | 9000 m |
| `GPS_NAV_MODE_SEA` | Tàu thuyền | 25 m/s | 500 m |
| `GPS_NAV_MODE_AIRBORNE_1G` | Máy bay nhẹ | 100 m/s | 50000 m |
| `GPS_NAV_MODE_AIRBORNE_2G` | Drone | 250 m/s | 50000 m |
| `GPS_NAV_MODE_AIRBORNE_4G` | High-G | 500 m/s | 50000 m |

---

## Luồng hoạt động

### Sequence Diagram

```
┌─────────┐     ┌──────────┐     ┌─────────┐
│   App   │     │ GPS Drv  │     │ NEO-7M  │
└────┬────┘     └────┬─────┘     └────┬────┘
     │               │                │
     │  init()       │                │
     ├──────────────►│                │
     │               │ UART Config    │
     │               ├───────────────►│
     │               │                │
     │               │ UBX Config     │
     │               ├───────────────►│
     │               │◄───────────────┤ ACK
     │               │                │
     │◄──────────────┤ return true    │
     │               │                │
     │  read()       │                │
     ├──────────────►│                │
     │               │ UART Read      │
     │               ├───────────────►│
     │               │◄───────────────┤ NMEA Data
     │               │                │
     │               │ Parse NMEA     │
     │               │ (GGA/RMC/VTG)  │
     │               │                │
     │◄──────────────┤ gps_data_t     │
     │               │                │
```

### State Machine

```
              ┌─────────────┐
              │  UNINIT     │
              └──────┬──────┘
                     │ init()
                     ▼
              ┌─────────────┐
     ┌───────►│ NO_FIX      │◄───────┐
     │        └──────┬──────┘        │
     │               │ satellites    │ timeout/
     │               │ acquired      │ signal lost
     │               ▼               │
     │        ┌─────────────┐        │
     │        │ ACQUIRING   │────────┘
     │        └──────┬──────┘
     │               │ fix obtained
     │               ▼
     │        ┌─────────────┐
     │        │  FIX_2D     │
     │        └──────┬──────┘
     │               │ altitude OK
     │               ▼
     │        ┌─────────────┐
     └────────│  FIX_3D     │
   cold_start └─────────────┘
```

---

## Tích hợp với FreeRTOS

### Task GPS đọc liên tục

```c
#include "gps_neo7m.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static QueueHandle_t gps_queue;

void gps_task(void *pvParameters) {
    gps_data_t data;

    // Khởi tạo GPS
    if (!gps_neo7m_init()) {
        ESP_LOGE("GPS", "Init failed!");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        // Đọc dữ liệu với timeout 1000ms
        if (gps_neo7m_read(&data, 1000)) {
            if (data.valid) {
                // Gửi dữ liệu qua queue
                xQueueSend(gps_queue, &data, 0);

                ESP_LOGI("GPS", "Lat: %.6f, Lon: %.6f, Sats: %d",
                         data.latitude, data.longitude, data.satellites_used);
            }
        }

        // Yield để task khác chạy
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    // Tạo queue cho GPS data
    gps_queue = xQueueCreate(5, sizeof(gps_data_t));

    // Tạo GPS task với stack 4KB
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}
```

---

## Build và Test

### Build

```bash
# Từ thư mục project
cd E:/New folder/Rainguard_Firmware

# Build project
idf.py build

# Flash và monitor
idf.py -p COMx flash monitor
```

### Cấu hình menuconfig

Nếu cần thay đổi UART pins hoặc baud rate, chỉnh sửa trong `pin_config.h`:

```c
#define GPS_TX_PIN 41         // ESP32 TX -> GPS RX
#define GPS_RX_PIN 42         // ESP32 RX <- GPS TX
#define GPS_UART_NUM UART_NUM_1
#define GPS_BAUD_RATE 9600
```

### Debug Commands

Sử dụng ESP-IDF monitor để debug:

```
// Trong code, gọi debug function
gps_neo7m_debug_status();
gps_neo7m_debug_satellites();
```

Output mẫu:
```
I (1234) gps_neo7m: === GPS NEO-7M DEBUG STATUS ===
I (1234) gps_neo7m: Initialized: YES
I (1234) gps_neo7m: Configuration:
I (1234) gps_neo7m:   UART Port: UART1
I (1234) gps_neo7m:   Baud Rate: 9600
I (1234) gps_neo7m:   Update Rate: 1 Hz
I (1234) gps_neo7m:   GLONASS: ON
I (1234) gps_neo7m: Statistics:
I (1234) gps_neo7m:   Total Reads: 100
I (1234) gps_neo7m:   Successful: 98
I (1234) gps_neo7m:   Valid Fixes: 95
I (1234) gps_neo7m:   TTFF: 28000 ms
```

---

## Xử lý lỗi

### Lỗi thường gặp

| Lỗi | Nguyên nhân | Giải pháp |
|-----|-------------|-----------|
| Không nhận được dữ liệu | Đấu sai TX/RX | Hoán đổi TX↔RX |
| Dữ liệu rác | Sai baud rate | Kiểm tra baud rate = 9600 |
| Checksum error | Nhiễu tín hiệu | Kiểm tra nguồn, dây nối |
| No fix | Che khuất tín hiệu | Di chuyển ra ngoài trời |
| Fix không ổn định | HDOP cao | Đợi thêm vệ tinh |

### Code xử lý lỗi

```c
gps_data_t data;
gps_stats_t stats;

if (!gps_neo7m_read(&data, 2000)) {
    gps_neo7m_get_stats(&stats);

    if (stats.successful_reads == 0) {
        ESP_LOGE("GPS", "No data - check wiring!");
    } else if (stats.checksum_errors > stats.successful_reads / 10) {
        ESP_LOGW("GPS", "High checksum error rate - check connections");
    }

    // Thử cold start nếu không có fix quá lâu
    if (gps_neo7m_time_since_fix() > 120000) { // 2 phút
        ESP_LOGW("GPS", "Attempting cold start...");
        gps_neo7m_cold_start();
    }
}
```

---

## Tối ưu hóa

### Tiết kiệm năng lượng

```c
// Giảm update rate khi không cần thiết
gps_neo7m_set_update_rate(1);

// Sử dụng power save mode khi đứng yên
if (speed < 0.5) {
    gps_neo7m_set_power_mode(GPS_POWER_SAVE);
} else {
    gps_neo7m_set_power_mode(GPS_POWER_FULL);
}
```

### Cải thiện TTFF

1. **Bật AssistNow Autonomous**: Dự đoán vị trí vệ tinh offline
2. **Giữ nguồn backup**: Duy trì RTC và almanac
3. **Sử dụng hot start**: Khi tắt GPS ngắn (<4h)

### Cải thiện độ chính xác

1. **Bật SBAS**: Hiệu chỉnh sai số ionosphere
2. **Bật GLONASS**: Tăng số vệ tinh khả dụng
3. **Chọn nav mode phù hợp**: Automotive cho xe, Sea cho tàu

---

## Tham khảo

- [u-blox NEO-7 Data Sheet (UBX-13003830)](https://content.u-blox.com/sites/default/files/products/documents/NEO-7_DataSheet_%28UBX-13003830%29.pdf)
- [u-blox 7 Receiver Description (GPS.G7-SW-12001)](https://www.u-blox.com/en/docs/UBX-13003221)
- [MAX-7/NEO-7 Hardware Integration Manual (UBX-13003704)](https://www.u-blox.com/en/docs/UBX-13003704)
- [ESP-IDF UART Driver Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/api-reference/peripherals/uart.html)

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-01-17 | Initial release for NEO-7M |

---

**Author**: RainGuard Firmware Team  
**License**: MIT
