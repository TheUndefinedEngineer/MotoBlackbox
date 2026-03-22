import struct
import pandas as pd

# Record sizes in bytes: must match the packed structs written by the firmware
IMU_SIZE = 16   # 6× int16 + 1× uint32  (hhhhhhI)
GPS_SIZE = 28   # 3× float + 12-byte time string + uint32  (fff12sI)

imu_rows = []
gps_rows = []
imu_count = 0
gps_count = 0
skip_count = 0

with open("log.bin", "rb") as f:
    while True:
        type_byte = f.read(1)
        if not type_byte:
            break  # clean end of file

        record_type = type_byte[0]

        if record_type == 0x01:  # IMU record
            data = f.read(IMU_SIZE)
            if len(data) < IMU_SIZE:
                break  # truncated record at end of file

            try:
                ax, ay, az, gx, gy, gz, ts = struct.unpack("<hhhhhhI", data)

                # Skip zero-initialised slots that were never written by the firmware
                if ts == 0 and ax == 0 and ay == 0 and az == 0:
                    continue

                imu_rows.append((ax, ay, az, gx, gy, gz, ts))
                imu_count += 1
            except struct.error:
                break

        elif record_type == 0x02:  # GPS record
            data = f.read(GPS_SIZE)
            if len(data) < GPS_SIZE:
                break

            try:
                lat, lon, spd, time_bytes, ts = struct.unpack("<fff12sI", data)
                time_str = time_bytes.decode(errors="ignore").strip("\x00")
                gps_rows.append((lat, lon, spd, time_str, ts))
                gps_count += 1
            except struct.error:
                break

        elif record_type == 0x00:
            # Zero padding between sectors — skip forward to the next 512-byte boundary
            pos = f.tell()
            next_sector = ((pos + 511) // 512) * 512
            skip_bytes = next_sector - pos
            f.seek(skip_bytes, 1)   # seek relative to current position
            skip_count += 1

        else:
            # Unknown tag — skip the byte and continue scanning
            skip_count += 1
            continue

# Build DataFrames, remove duplicate timestamps (can occur if the firmware
# logged the same sector twice after a retry), then sort chronologically
imu_df = pd.DataFrame(imu_rows, columns=["ax", "ay", "az", "gx", "gy", "gz", "timestamp"])
imu_df = imu_df.drop_duplicates(subset=["timestamp"])
imu_df = imu_df.sort_values("timestamp")
imu_df.to_csv("imu.csv", index=False)

gps_df = pd.DataFrame(gps_rows, columns=["latitude", "longitude", "speed", "time", "timestamp"])
gps_df = gps_df.drop_duplicates(subset=["timestamp"])
gps_df = gps_df.sort_values("timestamp")
gps_df.to_csv("gps.csv", index=False)

print(f"Done. IMU records: {imu_count}, GPS records: {gps_count}, skipped: {skip_count}")
print(f"After dedup — IMU: {len(imu_df)}, GPS: {len(gps_df)}")
