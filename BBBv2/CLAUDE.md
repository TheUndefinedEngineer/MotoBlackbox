# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BBBv2 is an automotive crash-detection and data-logging firmware for the **STM32F401CCUx** (ARM Cortex-M4, 84 MHz, 256 KB Flash, 64 KB RAM). It continuously samples an IMU into a ring buffer, detects crash events via threshold logic, and writes pre/post-crash sensor data to an SD card. The system wakes on ignition and sleeps in STOP mode when ignition is off.

## Build & Flash

This project is built and flashed exclusively through **STM32CubeIDE** (Eclipse CDT). There is no standalone Makefile. To build:

1. Open STM32CubeIDE and import the project from this directory.
2. Build via **Project → Build Project** (or `Ctrl+B`). Compiler: GCC ARM Embedded, optimization `-O2`.
3. Flash via **Run → Debug** (OpenOCD + ST-Link).

Compiled artifacts land in `Debug/` (`.elf`, `.map`, `.list`). These are tracked by git but should not be edited manually.

To regenerate HAL/peripheral init code, open `BBBv2.ioc` in STM32CubeMX (bundled with CubeIDE) and click **Generate Code**. This will overwrite `Core/Src/main.c` inside the `USER CODE` blocks only — keep all application logic inside those blocks.

## Hardware

| Peripheral | Interface | Key Config |
|---|---|---|
| MPU-6050 (IMU) | I2C1 | 400 kHz, addr 0x68, ±16 g / ±2000 dps |
| NEO-6M (GPS) | USART2 + DMA | 9600 baud, RX-only, circular DMA |
| SSD1306 (OLED) | I2C1 | 128×64, shared bus with IMU |
| SD Card | SPI1 | SPI master, sector base @ 2048 |
| Ignition sense | PB9 / EXTI9 | Rising edge → wakeup from STOP |
| Sensor power | PB10 | Active-low rail control |

## Architecture

### Main Loop (`Core/Src/main.c`)

The entire application state machine lives in `main.c` (~795 lines). Control flow:

1. **Init** — HAL, clocks, peripherals, sensors, SD, RTC, OLED.
2. **Grace period** — 5-second window after startup where crash detection is suppressed.
3. **Sampling loop** — TIM2 fires every 1 ms; every 25 ms it sets `sample_request_flag`. The main loop polls this flag to read the MPU-6050 and write into the ring buffer.
4. **Crash detection** — Each IMU sample is checked against accel/gyro thresholds. Three consecutive over-threshold samples set `crash_trigger_flag`.
5. **Post-crash capture** — 80 more samples (2 s) are captured before the buffer is frozen.
6. **SD logging** — The frozen buffer (2400 IMU samples + interleaved GPS records) is written sector-by-sector to the SD card in type-tagged binary format.
7. **Power down** — On ignition-low (PB9 falling, 500 ms debounce), sensors are powered off (PB10 high) and the MCU enters STOP mode. EXTI9 rising edge wakes it.

### Ring Buffer

`buffer[BUFFER_SIZE]` holds `BUFFER_SIZE = 2400` entries of `sample_t` (60 seconds at 40 Hz). `write_index` advances modulo `BUFFER_SIZE`. On crash, `crash_index` marks the event position and `buffer_freeze_flag` stops further writes.

### Data Structures (`Core/Inc/data_types.h`)

```c
typedef struct {
    int16_t  ax, ay, az;   // raw accelerometer ADC counts
    int16_t  gx, gy, gz;   // raw gyroscope ADC counts
    uint32_t timestamp;    // system_time_ms at sample time
} sample_t;

typedef struct {
    float   latitude, longitude;
    float   speed;         // knots
    char    time[12];      // UTC time string
    uint8_t valid;         // 1 = GPS fix, 0 = no fix
} gps_data_t;
```

### SD Card Log Format

Records are type-tagged binary written in 512-byte sectors starting at sector 2048:
- `LOG_TYPE_IMU = 0x01` — `sample_t` payload
- `LOG_TYPE_GPS = 0x02` — `gps_data_t` payload
- GPS records are interleaved every 40 IMU samples (every 1 second of data).
- Base sector advances after each logged event to prevent overwrite.

### Drivers (`Drivers/`)

- **`mpu6050/`** — I2C burst-read of 14-byte register block (accel + temp + gyro). `MPU6050_Init()` verifies WHO_AM_I. Caller must handle I2C bus recovery on error.
- **`neo6m/`** — DMA circular buffer receiving NMEA sentences. `GPS_GetData()` performs a critical-section atomic copy to avoid torn reads. Parser extracts `$GPRMC` / `$GPGGA`.
- **`sdcard/`** — Bit-bang SPI SD protocol (CMD0/CMD8/ACMD41/CMD58 init sequence). Supports SDSC, SDHC, SDXC. `SD_Write()` retries up to 3 times. Base sector 2048 avoids the FAT area.
- **`ssd1306/`** — Framebuffer-based I2C OLED. Call `ssd1306_Fill()` + `ssd1306_UpdateScreen()` to push a frame.

### Interrupt Handlers (`Core/Src/stm32f4xx_it.c`)

- **TIM2** — 1 ms tick: increments `system_time_ms`, sets `sample_request_flag` every 25 ms.
- **EXTI9** — Ignition rising edge: sets `ignition_on = 1`, `wakeup_flag = 1`.
- **DMA1_Stream5** — USART2 RX idle-line: GPS sentence ready callback.

### Memory Layout

- Flash: `0x08000000` – `0x08040000` (256 KB)  
- RAM: `0x20000000` – `0x20010000` (64 KB)  
- Heap: 512 B, Stack: 1 KB (tight — avoid large stack allocations)

## Key Constraints

- **RAM is tight (64 KB).** The ring buffer alone occupies ~33.6 KB (`2400 × 14 bytes`). Be cautious adding large local arrays or stack-heavy recursion.
- **Do not edit inside CubeMX-generated sections** outside `/* USER CODE BEGIN */` / `/* USER CODE END */` markers in `main.c` — regenerating from `.ioc` will overwrite them.
- **I2C bus is shared** between MPU-6050 and SSD1306. Ensure I2C transactions are not interleaved.
- **GPS DMA is circular** — always use `GPS_GetData()` (atomic copy) rather than reading the DMA buffer directly.
- **SD base sector (2048)** is hardcoded to skip the FAT partition. Raw sector writes will corrupt a FAT filesystem if the card is also FAT-formatted.
