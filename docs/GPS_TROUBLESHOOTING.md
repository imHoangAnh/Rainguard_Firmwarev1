# GPS NEO-7M Module - Troubleshooting Guide & Usage

## Table of Contents
1. [Overview](#1-overview)
2. [Hardware Checklist](#2-hardware-checklist)
3. [Common Problems & Solutions](#3-common-problems--solutions)
4. [Code Architecture](#4-code-architecture)
5. [NMEA Protocol Reference](#5-nmea-protocol-reference)
6. [MQTT Payload Structure](#6-mqtt-payload-structure)
7. [Configuration Options](#7-configuration-options)
8. [Testing & Debugging](#8-testing--debugging)
9. [FAQ](#9-faq)

---

## 1. Overview

This document provides comprehensive guidance for troubleshooting and using the GPS NEO-7M module with the RainGuard Firmware on ESP32-S3.

### 1.1 Module Specifications
| Parameter | Value |
|-----------|-------|
| Module | u-blox NEO-7M or NEO-7M |
| Interface | UART (TTL 3.3V) |
| Default Baud Rate | 9600 bps |
| Update Rate | 1-5 Hz (default 1 Hz) |
| Time to First Fix (TTFF) | Cold: 27s, Warm: 27s, Hot: 1s |
| Accuracy | Position: 2.5m CEP, Velocity: 0.1 m/s |
| Output Protocol | NMEA 0183 |

### 1.2 Pin Configuration
From `pin_config.h`:
```c
// GPS Configuration (UART1)
#define GPS_TX_PIN      41    // ESP32 -> GPS RX (MTDI, JTAG repurposed)
#define GPS_RX_PIN      42    // GPS TX -> ESP32 (MTMS, JTAG repurposed)  
#define GPS_UART_NUM    UART_NUM_1
#define GPS_BAUD_RATE   9600
```

### 1.3 Wiring Diagram
```
ESP32-S3                  NEO-7M Module
---------                 -------------
GPIO41 (TX) ───────────── RX
GPIO42 (RX) ───────────── TX
3.3V       ───────────── VCC
GND        ───────────── GND
```

> ⚠️ **Warning**: GPIO41/42 are JTAG pins. JTAG debugging will not work when GPS is connected.

---

## 2. Hardware Checklist

Before troubleshooting software, verify hardware connections:

### 2.1 Power Supply
- [ ] NEO-7M VCC connected to 3.3V (or 5V if module has onboard regulator)
- [ ] GND properly connected
- [ ] Module LED blinking (indicates active operation)
- [ ] Check current draw: NEO-7M requires ~45mA during tracking

### 2.2 UART Connections
- [ ] TX/RX wires NOT crossed (ESP TX → GPS RX, GPS TX → ESP RX)
- [ ] Verify GPIO41 and GPIO42 are not used by other peripherals
- [ ] No loose connections or cold solder joints

### 2.3 Antenna
- [ ] Active antenna connected with SMA/uFL connector tight
- [ ] Antenna has clear view of sky (outdoor or near window)
- [ ] Antenna placed away from metal objects and RF interference
- [ ] If using passive antenna, performance may be degraded indoors

### 2.4 First-Time Startup
- GPS cold start requires **2-15 minutes** for first fix
- Ensure GPS is in outdoor environment during initial testing
- LED blinking pattern:
  - Continuous blink: Searching for satellites
  - Single blink/second: Valid GPS fix acquired

---

## 3. Common Problems & Solutions

### 3.1 Problem: GPS returns all zeros
```json
"gps": {
    "latitude": 0,
    "longitude": 0,
    "altitude": 0,
    "speed": 0,
    "valid": false
}
```

**Possible Causes & Solutions:**

| Cause | Solution |
|-------|----------|
| No satellite fix | Move to outdoor location with clear sky view |
| Antenna not connected | Check antenna connector, ensure it clicks in place |
| Wrong baud rate | Verify GPS_BAUD_RATE is 9600 (NEO-7M default) |
| TX/RX swapped | GPIO41 should connect to GPS RX, GPIO42 to GPS TX |
| UART conflict | Check UART1 is not used by other components |
| Cold start | Wait 2-15 minutes for first fix |

### 3.2 Problem: Intermittent data / No NMEA sentences

**Debug Steps:**
1. Enable debug logging:
   ```c
   // In sdkconfig or menuconfig
   CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
   ```

2. Check raw UART data:
   ```c
   // Monitor serial output for:
   // "GPS Raw Data (xxx bytes): $GPRMC,..."
   ```

3. Verify UART is receiving data:
   ```c
   // Look for log: "GPS initialized successfully (UART1, 9600 baud)"
   ```

**Solutions:**
- Power cycle the GPS module
- Increase timeout: `gps_neo7m_read(&data, 2000)` (2 seconds)
- Check for electromagnetic interference
- Try different UART pins if possible

### 3.3 Problem: Parser errors / Checksum failures

```
Checksum failed: $GPRMC,123456...
```

**Solutions:**
- Check for electrical noise on UART lines
- Add capacitor (100nF) near GPS module VCC
- Shorten UART cable length
- Ensure stable power supply

### 3.4 Problem: Valid = false but coordinates exist

This indicates GPS has position but fix quality is insufficient.

**Solutions:**
- Wait for more satellites (need 4+ for valid fix)
- Check HDOP value (should be < 5.0 for good accuracy)
- Improve antenna placement

### 3.5 Problem: Outdated date/time

NEO-7M without battery backup loses time when powered off.

**Solutions:**
- Add backup battery to GPS module
- Use NTP time synchronization when WiFi is available
- Store last known time in NVS

---

## 4. Code Architecture

### 4.1 Module Overview

```
gps_neo7m.c
├── Initialization (gps_neo7m_init)
│   ├── UART driver installation
│   ├── Pin configuration  
│   └── Buffer flush
│
├── Data Reading (gps_neo7m_read)
│   ├── UART read with timeout
│   ├── NMEA sentence extraction
│   ├── Checksum validation
│   └── Sentence parsing (GGA, RMC, VTG)
│
├── Parsing Functions
│   ├── parse_gga_sentence() - Position, altitude, satellites
│   ├── parse_rmc_sentence() - Position, speed, time, date
│   └── parse_vtg_sentence() - Course, ground speed
│
└── UBX Configuration
    ├── gps_neo7m_set_update_rate()
    ├── gps_neo7m_set_nav_mode()
    ├── gps_neo7m_set_power_mode()
    └── gps_neo7m_cold_start()
```

### 4.2 Data Flow

```
NEO-7M Module
     │
     │ UART (9600 baud, 8N1)
     ▼
┌────────────────────────────────────┐
│   uart_read_bytes()                │
│   Read raw bytes into buffer       │
└────────────────────────────────────┘
     │
     ▼
┌────────────────────────────────────┐
│   Extract NMEA Sentences           │
│   Find '$' start, '\r\n' end       │
└────────────────────────────────────┘
     │
     ▼
┌────────────────────────────────────┐
│   Validate Checksum                │
│   XOR chars between '$' and '*'    │
└────────────────────────────────────┘
     │
     ▼
┌────────────────────────────────────┐
│   Parse Sentence Type              │
│   GP/GN/GL + GGA/RMC/VTG          │
└────────────────────────────────────┘
     │
     ▼
┌────────────────────────────────────┐
│   Extract Fields                   │
│   Split by ',' delimiter           │
└────────────────────────────────────┘
     │
     ▼
┌────────────────────────────────────┐
│   Merge Data (RMC + GGA)           │
│   Create gps_data_t structure      │
└────────────────────────────────────┘
     │
     ▼
┌────────────────────────────────────┐
│   Format JSON & Publish MQTT       │
└────────────────────────────────────┘
```

### 4.3 Key Data Structures

```c
// GPS data structure
typedef struct {
    float latitude;          // Decimal degrees (+ = North)
    float longitude;         // Decimal degrees (+ = East)
    float altitude;          // Meters above MSL
    float speed;             // km/h (converted from knots)
    float course;            // Degrees (0-360)
    uint8_t satellites;      // Satellites in use
    float hdop;              // Horizontal DOP
    bool valid;              // True if GPS fix is valid
    gps_fix_type_t fix_type; // NONE/2D/3D
    gps_time_t utc_time;     // UTC time (H:M:S.ms)
    gps_date_t utc_date;     // UTC date (D/M/Y)
} gps_data_t;
```

### 4.4 GNSS Prefix Support

The parser supports multiple GNSS constellation prefixes:
- `$GP` - GPS (USA)
- `$GN` - Multi-GNSS combined
- `$GL` - GLONASS (Russia)
- `$GA` - Galileo (Europe)
- `$BD` - BeiDou (China)

---

## 5. NMEA Protocol Reference

### 5.1 GGA - GPS Fix Data
```
$GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh

Field  Description
0      Time (hhmmss.ss)
1      Latitude (ddmm.mmmmm)
2      N/S Hemisphere
3      Longitude (dddmm.mmmmm)
4      E/W Hemisphere  
5      Fix quality (0=invalid, 1=GPS, 2=DGPS, 4=RTK Fixed)
6      Number of satellites (00-12)
7      HDOP (0.5-99.9)
8      Altitude above MSL (meters)
9      Altitude units (M)
10     Geoid separation (meters)
11     Geoid units (M)
...
```

### 5.2 RMC - Recommended Minimum
```
$GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh

Field  Description
0      Time (hhmmss.ss)
1      Status (A=active, V=void)
2      Latitude (ddmm.mmmmm)
3      N/S Hemisphere
4      Longitude (dddmm.mmmmm)
5      E/W Hemisphere
6      Speed over ground (knots)
7      Course over ground (degrees)
8      Date (ddmmyy)
9      Magnetic variation
10     E/W direction of variation
```

### 5.3 VTG - Course and Speed
```
$GPVTG,x.x,T,x.x,M,x.x,N,x.x,K*hh

Field  Description
0      Course (true degrees)
1      T (true)
2      Course (magnetic degrees)
3      M (magnetic)
4      Speed (knots)
5      N (knots)
6      Speed (km/h)
7      K (km/h)
```

---

## 6. MQTT Payload Structure

### 6.1 Optimized GPS Payload

The GPS data in MQTT is optimized to include only essential fields:

```json
{
    "device_id": "rainguard-001",
    "timestamp": 1234567890,
    "gps": {
        "latitude": 10.762622,
        "longitude": 106.660172,
        "altitude": 15.5,
        "speed": 2.35,
        "utc_time": "2026-01-17T10:30:45.123Z",
        "valid": true
    }
}
```

### 6.2 Field Descriptions

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| latitude | float | degrees | Positive = North, Negative = South |
| longitude | float | degrees | Positive = East, Negative = West |
| altitude | float | meters | Height above mean sea level |
| speed | float | km/h | Ground speed |
| utc_time | string | ISO8601 | UTC timestamp from GPS |
| valid | boolean | - | True if GPS has valid fix |

### 6.3 Payload Size Comparison

| Version | Fields | Est. Size |
|---------|--------|-----------|
| Original | 8 fields (lat, lon, alt, speed, course, sats, hdop, valid) | ~150 bytes |
| Optimized | 6 fields (lat, lon, alt, speed, utc_time, valid) | ~120 bytes |

Reduction: **~20% smaller payload**

---

## 7. Configuration Options

### 7.1 Update Rate Configuration

```c
// Set GPS update rate (1-5 Hz recommended for NEO-7M)
gps_neo7m_set_update_rate(5);  // 5 Hz = 200ms updates
```

### 7.2 Navigation Mode

```c
// Set navigation mode for specific use case
gps_neo7m_set_nav_mode(GPS_NAV_MODE_PORTABLE);     // General use
gps_neo7m_set_nav_mode(GPS_NAV_MODE_STATIONARY);   // Fixed location
gps_neo7m_set_nav_mode(GPS_NAV_MODE_PEDESTRIAN);   // Walking
gps_neo7m_set_nav_mode(GPS_NAV_MODE_AUTOMOTIVE);   // Vehicle
gps_neo7m_set_nav_mode(GPS_NAV_MODE_SEA);          // Marine
gps_neo7m_set_nav_mode(GPS_NAV_MODE_AIRBORNE);     // Aircraft
```

### 7.3 Power Management

```c
// Enable low power mode (backup mode)
gps_neo7m_set_power_mode(false);  

// Wake up from low power
gps_neo7m_set_power_mode(true);
```

### 7.4 Cold Start

```c
// Perform cold start (clears all saved data)
gps_neo7m_cold_start();
// Note: TTFF will be 2-15 minutes after cold start
```

---

## 8. Testing & Debugging

### 8.1 Enable Debug Logging

Add to `main.c`:
```c
// Set GPS logging to DEBUG level
esp_log_level_set("gps_neo7m", ESP_LOG_DEBUG);
```

### 8.2 Expected Debug Output

**No Fix:**
```
D gps_neo7m: GPS Raw Data (342 bytes): $GPRMC,123456.00,V,,,,,,,170126,,,N*7E...
D gps_neo7m: RMC parsed: Status=VOID (no fix)
D gps_neo7m: GPS: No fix (RMC=Y, GGA=Y, VTG=N, Sats=0)
```

**Valid Fix:**
```
D gps_neo7m: GPS Raw Data (425 bytes): $GPRMC,103045.00,A,1045.7573,N,10639.61...
D gps_neo7m: RMC parsed: Lat=10.762622, Lon=106.660172, Speed=2.35 km/h, Valid=YES
D gps_neo7m: GGA parsed: Lat=10.762622, Lon=106.660172, Alt=15.5, Sats=8
I gps_neo7m: GPS Fix: Lat=10.762622, Lon=106.660172, Alt=15.5m, Speed=2.35 km/h, Sats=8, HDOP=1.2, UTC=10:30:45
```

### 8.3 UART Monitor Tool

Use external tool to verify raw GPS output:
```bash
# On Linux/Mac
screen /dev/ttyUSB0 9600

# On Windows (use PuTTY or similar)
# Connect to COMx at 9600 baud
```

### 8.4 GPS Simulator

For indoor testing, use GPS simulator apps:
- **u-center** (Windows) - Official u-blox software
- **NMEA Generator** - Python script for simulated data

---

## 9. FAQ

### Q1: How long should I wait for GPS fix?
**A:** Cold start: 2-15 minutes. Hot start: <5 seconds. Ensure clear sky view.

### Q2: Can I use NEO-7M with 5V?
**A:** Yes, most modules have onboard LDO. VCC accepts 3-5V, but logic is 3.3V.

### Q3: Why does GPS work outdoor but not indoor?
**A:** GPS signals cannot penetrate buildings. Use near windows or outdoors.

### Q4: How accurate is NEO-7M?
**A:** Typical 2.5m CEP (50% of positions within 2.5m of true location).

### Q5: What is HDOP?
**A:** Horizontal Dilution of Precision. Lower is better. 
- <1: Ideal
- 1-2: Excellent  
- 2-5: Good
- 5-10: Moderate
- >10: Poor

### Q6: Can I increase update rate to 10 Hz?
**A:** NEO-7M max is 5 Hz. NEO-7M/8M support up to 10 Hz.

### Q7: GPS module LED not blinking?
**A:** Check power connection. LED indicates module is powered and searching.

### Q8: Why are some sentences missing checksum?
**A:** Corrupted data or incomplete sentence due to timing. The parser validates checksum and skips invalid sentences.

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 2.0.0 | 2026-01-17 | RainGuard Team | Complete rewrite with checksum validation, multi-GNSS support |
| 1.0.0 | 2026-01-01 | RainGuard Team | Initial version |

---

## References

- [u-blox NEO-6 Data Sheet](https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf)
- [NMEA 0183 Protocol Specification](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)
- [ESP-IDF UART Driver Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html)
