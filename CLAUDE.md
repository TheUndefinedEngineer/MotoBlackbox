# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BBBv2 is embedded firmware for a two-wheeler crash-detection and data-logging blackbox built on the **STM32F401CCUx** (Black Pill, ARM Cortex-M4, 84 MHz, 256 KB Flash, 64 KB RAM). It continuously samples an IMU into a 60-second ring buffer, detects crash events via consecutive-threshold logic, and writes pre/post-crash sensor data plus GPS fixes to an SD card in raw binary sector format.

## Repository Layout

```
BBBv2/             STM32CubeIDE project (firmware source)
  Core/Src/main.c  Application state machine (~795 lines)
  Core/Inc/        data_types.h, main.h, peripheral headers
  Drivers/         mpu6050/, neo6m/, sdcard/, ssd1306/ (custom drivers)
  Debug/           Compiled artifacts (.elf, .map, .list) — do not edit
  BBBv2.ioc        STM32CubeMX peripheral config
  CLAUDE.md        Detailed firmware-specific reference
Python Scripts/    decode-to-csv.py, plot.py (post-processing tools)
logs/              Raw binary dumps and decoded CSVs from real captures
Images/            Architecture diagrams and screenshots
```

## Build & Flash

Built exclusively through **STM32CubeIDE** (Eclipse CDT, GCC ARM Embedded, `-O2`). No standalone Makefile.

1. Open STM32CubeIDE, import the project from `BBBv2/`.
2. **Project → Build Project** (`Ctrl+B`).
3. **Run → Debug** to flash via OpenOCD + ST-Link.

To regenerate HAL/peripheral init from `.ioc`: open `BBBv2/BBBv2.ioc` in STM32CubeMX and click **Generate Code**. This rewrites `main.c` — all application logic must stay inside `/* USER CODE BEGIN */` / `/* USER CODE END */` markers or it will be lost.

## Hardware

| Peripheral | Interface | Key Config |
|---|---|---|
| MPU-6050 (IMU) | I2C1 (PB6/PB7) | 400 kHz, addr 0x68, ±16 g / ±2000 dps |
| NEO-6M (GPS) | USART2 + DMA (PA3) | 9600 baud, RX-only, circular DMA |
| SSD1306 (OLED) | I2C1 (PB6/PB7) | 128×64, shared bus with IMU |
| SD Card | SPI1 (PA4–PA7) | Bit-bang, base sector 2048 |
| Ignition sense | PB5 / EXTI | Rising edge → wakeup from STOP mode |
| Sensor power rail | PB10 | P-MOSFET, active-low |

## Architecture

The full application state machine lives in `BBBv2/Core/Src/main.c`:

1. **Init** — HAL, clocks, peripherals, sensors, SD card, RTC, OLED.
2. **Grace period** — 5-second startup window where crash detection is suppressed.
3. **Sampling loop** — TIM2 fires every 1 ms; every 25 ms it sets `sample_request_flag`. Main loop polls this to read MPU-6050 and write into the ring buffer at 40 Hz.
4. **Crash detection** — Each sample is checked: `ax²+ay²+az² ≥ 15,000,000` OR `gx²+gy²+gz² ≥ 3,000,000`. Three consecutive over-threshold samples set `crash_trigger_flag`.
5. **Post-crash capture** — 80 more samples (2 s) captured before the buffer freezes.
6. **SD logging** — Frozen buffer written sector-by-sector in type-tagged binary format. GPS records interleaved every 40 IMU samples (1 Hz).
7. **Power down** — On ignition-low (PB5 falling, 500 ms debounce), sensor rail off (PB10 high), MCU enters STOP mode; EXTI rising edge wakes it.

### Ring buffer

`buffer[2400]` of `sample_t` = ~33.6 KB in RAM (60 seconds at 40 Hz). `write_index` wraps modulo 2400. On crash, `crash_index` marks the event and `buffer_freeze_flag` halts further writes.

### SD log format

Raw 512-byte sectors from sector 2048 (avoids FAT area). Type-tagged records, little-endian:

| Tag | Type | Payload |
|---|---|---|
| `0x01` | IMU | `ax,ay,az,gx,gy,gz` (int16×6) + `timestamp` (uint32) — 16 bytes |
| `0x02` | GPS | `lat,lon,speed` (float×3) + `time[12]` + `timestamp` (uint32) — 28 bytes |
| `0x00` | Padding | Zero-fill to next 512-byte sector boundary |

### Drivers

- **`mpu6050/`** — I2C burst-read of 14-byte register block. `MPU6050_Init()` verifies WHO_AM_I. Caller handles I2C bus recovery on error.
- **`neo6m/`** — DMA circular buffer receiving NMEA. Always use `GPS_GetData()` (atomic copy) — never read the DMA buffer directly.
- **`sdcard/`** — Bit-bang SPI SD protocol (CMD0/CMD8/ACMD41/CMD58). Supports SDSC/SDHC/SDXC. `SD_Write()` retries up to 3 times.
- **`ssd1306/`** — Framebuffer I2C OLED. Call `ssd1306_Fill()` + `ssd1306_UpdateScreen()` per frame.

### Interrupt handlers (`Core/Src/stm32f4xx_it.c`)

- **TIM2** — 1 ms tick: increments `system_time_ms`, sets `sample_request_flag` every 25 ms.
- **EXTI (PB5)** — Ignition rising edge: sets `ignition_on = 1`, `wakeup_flag = 1`.
- **DMA1_Stream5** — USART2 RX idle-line: GPS sentence ready.

## Key Constraints

- **RAM is tight (64 KB).** The ring buffer alone is ~33.6 KB. Avoid large stack allocations or local arrays.
- **CubeMX sections** — Never edit outside `/* USER CODE BEGIN */` / `/* USER CODE END */` in `main.c`; regenerating from `.ioc` overwrites everything else.
- **I2C bus shared** between MPU-6050 and SSD1306 — do not interleave I2C transactions.
- **GPS DMA is circular** — always use `GPS_GetData()` for atomic reads.
- **Sector 2048 is hardcoded** — raw SD writes will corrupt a FAT filesystem if the card is also FAT-formatted.
- **Stack: 1 KB** — avoid deep call chains or large local buffers.

## Post-Processing Workflow

### 1. Extract raw log from SD card

```bash
sudo dd if=/dev/sda of=log.bin bs=512 skip=2048 count=200
```

Replace `/dev/sda` with the actual SD card device. `count=200` captures 200 sectors (100 KB); adjust if needed.

### 2. Decode binary log to CSV

```bash
cd "Python Scripts"
python decode-to-csv.py    # reads log.bin → writes imu.csv and gps.csv
```

Skips zero-padded slots, removes duplicate timestamps from SD write retries, and sorts chronologically.

### 3. Visualise and detect crashes

```bash
python plot.py imu.csv
```

Replicates the firmware detection algorithm exactly and renders a 4-panel plot:
- Accelerometer squared magnitude vs threshold
- Gyroscope squared magnitude vs threshold
- `crash_counter` trace (ramps to 3 at trigger)
- Binary crash/normal state

Detected crash windows are shaded across all panels; peak magnitudes printed to console.

Dependencies: `pandas`, `matplotlib` (Python 3).

> **Note:** `README.md` refers to these scripts as `parse_log.py` and `visualise.py` — those are outdated names. The actual files are `decode-to-csv.py` and `plot.py`.
