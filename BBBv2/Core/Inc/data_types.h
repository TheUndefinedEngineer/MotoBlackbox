#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>

/* One IMU sample captured from the MPU-6050.
   Raw 16-bit ADC counts; scale depends on configured FSR. */
typedef struct {
    int16_t  ax, ay, az;   /* accelerometer X, Y, Z */
    int16_t  gx, gy, gz;   /* gyroscope X, Y, Z     */
    uint32_t timestamp;    /* system_time_ms at capture */
} sample_t;

/* Latest parsed GPS fix from a single NMEA sentence. */
typedef struct {
    float   latitude;
    float   longitude;
    float   speed;         /* speed over ground, knots */
    char    time[12];      /* UTC time string from NMEA, e.g. "123519.00" */
    uint8_t valid;         /* 1 = NMEA status 'A' (fix), 0 = 'V' (no fix) */
} gps_data_t;

#endif
