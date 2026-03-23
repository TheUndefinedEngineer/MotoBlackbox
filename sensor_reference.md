## Sensor Data Reference

### IMU — MPU-6050 (I2C, 40 Hz)

| Field | Type | Scale / Unit | Used For |
|---|---|---|---|
| `ax, ay, az` | int16 | ±16 g, LSB = 2048 counts/g | Crash: ax²+ay²+az² ≥ 15,000,000 · OLED display · SD log (0x01) |
| `gx, gy, gz` | int16 | ±2000 dps, LSB = 16.4 counts/dps | Crash: gx²+gy²+gz² ≥ 3,000,000 · OLED display · SD log (0x01) |
| `timestamp`  | uint32 | ms — system_time_ms at capture | SD log (0x01) · post-crash 2 s window counter |

### GPS — NMEA (UART DMA, 1 Hz)

| Field | Type | Unit | Used For |
|---|---|---|---|
| `latitude` | float | Decimal degrees (±90) | SD log (0x02, fix only) |
| `longitude` | float | Decimal degrees (±180) | SD log (0x02, fix only) |
| `speed` | float | Knots | SD log (0x02, fix only) |
| `time` | char[12] | UTC string e.g. `123519.00` | OLED line 3 · SD log (0x02) |
| `valid` | uint8 | 1 = fix (NMEA 'A'), 0 = no fix ('V') | OLED V: field · gates whether 0x02 record is written |

### RTC (LSE 32.768 kHz, snapshot at 1 Hz)

| Field | Type | Unit | Used For |
|---|---|---|---|
| `Hours, Minutes, Seconds` | uint8 | 24-hour BIN | OLED T: line |
| `Date, Month, Year` | uint8 | BIN, Year offset from 2000 | OLED D: line |

### Ignition (GPIO EXTI)

| Signal | Level | Used For |
|---|---|---|
| `Ignition_Pin` | Rising edge | EXTI wakeup from STOP mode |
| `Ignition_Pin` | Low > 500 ms | Sensor rail off + enter STOP mode |
