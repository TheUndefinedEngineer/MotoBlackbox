# Two-Wheeler Crash Detection & Blackbox System

A low-power embedded blackbox for two-wheelers that continuously monitors motion,
detects crash events, and logs IMU and GPS data to an SD card for post-analysis.

---

## Hardware

| Component | Part | Interface |
|---|---|---|
| Microcontroller | STM32F401CCU6 (Black Pill) | — |
| IMU | MPU-6050 | I2C @ 400 kHz |
| Storage | SD card module | SPI |
| GPS | NMEA module (9600 baud) | UART + DMA |
| Display | SSD1306 OLED (128×64) | I2C |
| Sensor power rail | P-MOSFET switch | GPIO (active-low) |

---

## How It Works

### Continuous recording

The IMU is sampled at **40 Hz**. Each sample is written into a **2400-sample
(60-second) ring buffer** in RAM. The buffer overwrites itself continuously so
the most recent 60 seconds are always available.

Only samples that have actually been written are ever logged — a `samples_written`
counter tracks the true fill level so uninitialised slots are never included if a
crash occurs before the buffer has been fully populated.

### Crash detection

Every sample's squared vector magnitude is compared against fixed thresholds
(no square root required):

```
accel_mag² = ax² + ay² + az²  ≥  15,000,000
gyro_mag²  = gx² + gy² + gz²  ≥   3,000,000
```

Either condition alone is sufficient. **Three consecutive** above-threshold
samples (~75 ms) must occur before a crash is confirmed, which rejects
single-sample noise spikes.

### Post-crash capture

After confirmation, the system collects a further **2 seconds** (80 samples)
of post-crash data before freezing the buffer.

### SD logging

The frozen buffer — up to 60 seconds of pre-crash data plus 2 seconds
post-crash — is written to the SD card as a tagged binary log. A GPS record is
prepended and then interleaved every 40 IMU samples (once per second) to
geo-reference the event.

After logging completes, `base_sector` is advanced past the maximum possible
log size so subsequent crash events are written to fresh sectors without
overwriting earlier logs.

### IMU fault tolerance

If an I2C read fails, the last successfully read sample is reused so the ring
buffer always contains plausible data. A 10 ms read-timeout watchdog forces an
I2C peripheral re-init if the bus stalls. The OLED refresh also monitors I2C
state and performs a full DeInit/re-init + display re-init if the bus is found
busy outside of a transaction.

---

## Log Format

Data is written as raw **512-byte sectors** starting at sector 2048 (clear of
any FAT structures). Each record is prefixed with a 1-byte type tag.

| Tag | Type | Payload | Size |
|---|---|---|---|
| `0x01` | IMU sample | `ax, ay, az, gx, gy, gz` (int16 ×6) + `timestamp` (uint32) | 16 bytes |
| `0x02` | GPS fix | `lat, lon, speed` (float ×3) + `time[12]` + `timestamp_ms` (uint32) | 28 bytes |
| `0x00` | Padding | Zero-fill to next 512-byte boundary | — |

All multi-byte fields are **little-endian**. IMU values are raw ADC counts at
±16 g / ±2000 dps full-scale. GPS coordinates are decimal degrees. GPS records
are only written when a valid satellite fix is active at the time of the crash.

---

## Post-Processing Tools

Both tools are in `Python Scripts/` and require Python 3 with `pandas` and
`matplotlib`.

### 1. Binary parser — `decode-to-csv.py`

Reads `log.bin` and produces `imu.csv` and `gps.csv`:

```
cd "Python Scripts"
python decode-to-csv.py
```

Output columns:

```
imu.csv  →  ax, ay, az, gx, gy, gz, timestamp
gps.csv  →  latitude, longitude, speed, time, timestamp
```

Duplicate timestamps (from SD write retries) are removed automatically and
records are sorted chronologically. Zero-padded sectors are skipped by seeking
to the next 512-byte boundary.

### 2. Crash visualiser — `plot.py`

Replicates the firmware detection algorithm exactly and plots four panels:

```
python plot.py imu.csv
```

| Panel | Content |
|---|---|
| 1 | Accelerometer squared magnitude vs threshold |
| 2 | Gyroscope squared magnitude vs threshold |
| 3 | `crash_counter` trace (ramps to 3, then triggers) |
| 4 | Binary crash / normal state |

Detected crash windows are shaded across all panels with peak magnitude
values printed to the console. Gaps in the timestamp sequence are rendered
as breaks in the plot rather than interpolated lines.

---

## OLED Display

During normal operation the display refreshes at **5 Hz** and shows:

```
A: [ax]  [ay]  [az]
G: [gx]  [gy]  [gz]
V:[fix] T:[UTC time]
T: HH:MM:SS
D: DD/MM/YY
UP: [uptime s]
```

On crash detection the screen immediately switches to a **CRASH!** alert and
holds until logging completes, at which point it shows **Log Saved** and
returns to the normal status view.

---

## RTC

The RTC runs from the internal **LSI** oscillator (~32 kHz). On first boot a
hardcoded default time is loaded into the RTC and a sentinel value is written to
backup register `BKP_DR0`; subsequent reboots detect this sentinel and leave the
running time intact.

---

## Pin Assignment

| Signal | Pin | Notes |
|---|---|---|
| I2C SDA | PB7 | MPU-6050 + SSD1306 (shared bus) |
| I2C SCL | PB6 | MPU-6050 + SSD1306 (shared bus) |
| SPI MOSI | PA7 | SD card |
| SPI MISO | PA6 | SD card |
| SPI SCK | PA5 | SD card |
| SD CS | PA4 | Active-low |
| UART2 RX | PA3 | GPS NMEA (DMA, circular) |
| Sensor toggle | PB10 | P-MOSFET gate, active-low |

---

## Building

The project targets the STM32F401CCU6 and is configured for an **84 MHz** system
clock derived from a 25 MHz HSE crystal via PLL (PLLM=25, PLLN=168, PLLP=2).

Open in **STM32CubeIDE** and build normally (`Ctrl+B`). One external library
is required:

| Library | Source | Purpose |
|---|---|---|
| stm32-ssd1306 | https://github.com/afiskon/stm32-ssd1306 | SSD1306 OLED driver |

Copy `ssd1306.c`, `ssd1306.h`, and `ssd1306_fonts.h` from that repository into
`Drivers/ssd1306/`. No other external dependencies are needed.

---

## Known Limitations

- Threshold values were tuned by bench shaking and have not been validated
  against real-world crash data.
- The SD card runs at SPI prescaler /256 (~328 kHz). Logging a full 60-second
  buffer takes several seconds.
- GPS coordinates are only logged if a satellite fix was active at the time of
  the crash. No dead-reckoning or last-known position fallback.
- The RTC default time is hardcoded and must be corrected in firmware for
  accurate absolute timestamps; it is not synchronised from GPS automatically.

---

## Possible Improvements

- Sync RTC automatically from GPS NMEA time fields
- Switch SD SPI clock to a higher prescaler after initialisation to reduce logging time
- Add GSM/BLE module to transmit crash location as an alert
- Implement a complementary or Kalman filter for more robust crash classification
- Validate and tune thresholds against real accident data
- Re-add low-power STOP mode with ignition-line wakeup for always-on deployment

---
