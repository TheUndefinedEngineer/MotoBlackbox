import matplotlib.pyplot as plt
import csv
import sys
import os


def read_csv(filepath):
    samples = []
    with open(filepath, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ax = float(row["ax"])
                ay = float(row["ay"])
                az = float(row["az"])
                gx = float(row["gx"])
                gy = float(row["gy"])
                gz = float(row["gz"])
                ts = float(row["timestamp"])
                samples.append((ax, ay, az, gx, gy, gz, ts))
            except (ValueError, KeyError):
                continue  # skip malformed or incomplete rows silently
    return samples


def detect_crashes(accel_mag_sq, gyro_mag_sq,
                   accel_threshold, gyro_threshold,
                   consecutive_required=5):
    """
    Mirrors the firmware crash detection logic exactly:

        if (accel_mag >= ACCEL_THRESHOLD || gyro_mag >= GYRO_THRESHOLD)
            crash_counter++
            if (crash_counter >= 5)  → crash triggered
        else
            crash_counter = 0

    Returns:
        crash_events  — list of sample indices where crash was confirmed
        counter_trace — crash_counter value at every sample (for plotting)
    """
    crash_events  = []
    counter_trace = []
    crash_counter = 0

    for i in range(len(accel_mag_sq)):
        above = (accel_mag_sq[i] >= accel_threshold or
                 gyro_mag_sq[i]  >= gyro_threshold)

        if above:
            crash_counter += 1
            if crash_counter >= consecutive_required:
                crash_events.append(i)
        else:
            crash_counter = 0  # any below-threshold sample resets the counter

        counter_trace.append(crash_counter)

    return crash_events, counter_trace


def find_crash_windows(crash_events, gap=10):
    """Merge consecutive confirmed-crash indices into (start, end) windows.
    Indices within `gap` samples of each other are treated as one event."""
    if not crash_events:
        return []
    windows = []
    start = crash_events[0]
    end   = crash_events[0]
    for idx in crash_events[1:]:
        if idx - end <= gap:
            end = idx          # extend current window
        else:
            windows.append((start, end))
            start = idx        # begin a new window
            end   = idx
    windows.append((start, end))
    return windows


def main(csv_path):
    if not os.path.exists(csv_path):
        print(f"Error: file not found - {csv_path}")
        sys.exit(1)

    samples = read_csv(csv_path)
    if not samples:
        print("No valid samples found in CSV.")
        sys.exit(1)

    # Thresholds copied verbatim from firmware (squared magnitude, raw ADC units)
    # OR condition: either sensor alone is sufficient to increment the counter
    ACCEL_THRESHOLD    = 15_000_000
    GYRO_THRESHOLD     =  3_000_000
    CONSECUTIVE_NEEDED = 5   # ~125 ms at 40 Hz

    accel_mag_sq = []
    gyro_mag_sq  = []
    timestamps   = []

    for ax, ay, az, gx, gy, gz, ts in samples:
        if ax == ay == az == gx == gy == gz == 0:
            continue  # discard uninitialised buffer slots (same filter as parser)
        accel_mag_sq.append(ax*ax + ay*ay + az*az)
        gyro_mag_sq.append(gx*gx + gy*gy + gz*gz)
        timestamps.append(ts)

    crash_events, counter_trace = detect_crashes(
        accel_mag_sq, gyro_mag_sq,
        ACCEL_THRESHOLD, GYRO_THRESHOLD,
        CONSECUTIVE_NEEDED
    )

    crash_windows = find_crash_windows(crash_events)

    # Console summary
    print(f"Total samples         : {len(accel_mag_sq)}")
    print(f"Crash events detected : {len(crash_windows)}")
    for i, (s, e) in enumerate(crash_windows, 1):
        peak_a = max(accel_mag_sq[s:e+1])
        peak_g = max(gyro_mag_sq[s:e+1])
        print(f"  Event {i}: samples {s}-{e} "
              f"| ts {timestamps[s]:.0f}-{timestamps[e]:.0f} ms "
              f"| peak accel_sq {peak_a:,.0f} (thresh {ACCEL_THRESHOLD:,}) "
              f"| peak gyro_sq {peak_g:,.0f} (thresh {GYRO_THRESHOLD:,})")
    if not crash_windows:
        print("No crash detected in this log.")

    # 4-panel plot: accel magnitude, gyro magnitude, counter trace, binary state
    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True,
                             gridspec_kw={"height_ratios": [2, 2, 1, 1]})
    fig.suptitle("IMU Crash Detection  —  firmware-accurate",
                 fontsize=13, fontweight="bold", y=0.99)

    COLOR_ACCEL  = "#378ADD"
    COLOR_GYRO   = "#1D9E75"
    COLOR_THRESH = "#E24B4A"
    COLOR_FILL   = "#FAEEDA"
    COLOR_BORDER = "#E24B4A"
    COLOR_COUNT  = "#BA7517"

    def shade_crashes(ax):
        """Shade crash windows on a subplot with a fill and a dashed start line."""
        for i, (s, e) in enumerate(crash_windows):
            ax.axvspan(s, e, color=COLOR_FILL, alpha=0.7,
                       label="Crash window" if i == 0 else "")
            ax.axvline(x=s, color=COLOR_BORDER, linewidth=1.5,
                       linestyle="--", label="Crash start" if i == 0 else "")

    # Panel 1 — accelerometer squared magnitude vs threshold
    ax1 = axes[0]
    ax1.plot(accel_mag_sq, color=COLOR_ACCEL, linewidth=0.7, label="Accel mag²")
    ax1.axhline(ACCEL_THRESHOLD, color=COLOR_THRESH, linestyle=":",
                linewidth=1.2, label=f"Threshold ({ACCEL_THRESHOLD:,})")
    shade_crashes(ax1)
    ax1.set_ylabel("Accel mag²", fontsize=10)
    ax1.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax1.grid(True, linewidth=0.3, alpha=0.4)
    ax1.set_title("Accelerometer (squared)", fontsize=10, loc="left", pad=4)

    # Panel 2 — gyroscope squared magnitude vs threshold
    ax2 = axes[1]
    ax2.plot(gyro_mag_sq, color=COLOR_GYRO, linewidth=0.7, label="Gyro mag²")
    ax2.axhline(GYRO_THRESHOLD, color=COLOR_THRESH, linestyle=":",
                linewidth=1.2, label=f"Threshold ({GYRO_THRESHOLD:,})")
    shade_crashes(ax2)
    ax2.set_ylabel("Gyro mag²", fontsize=10)
    ax2.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax2.grid(True, linewidth=0.3, alpha=0.4)
    ax2.set_title("Gyroscope (squared)", fontsize=10, loc="left", pad=4)

    # Panel 3 — firmware crash_counter replica; ramps to CONSECUTIVE_NEEDED then triggers
    ax3 = axes[2]
    ax3.plot(counter_trace, color=COLOR_COUNT, linewidth=0.8, label="crash_counter")
    ax3.axhline(CONSECUTIVE_NEEDED, color=COLOR_THRESH, linestyle=":",
                linewidth=1.2, label=f"Trigger level ({CONSECUTIVE_NEEDED})")
    ax3.set_ylabel("Counter", fontsize=10)
    ax3.set_yticks(range(0, CONSECUTIVE_NEEDED + 2))
    ax3.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax3.grid(True, linewidth=0.3, alpha=0.4)
    ax3.set_title("Firmware crash_counter  (triggers at 5 consecutive samples)",
                  fontsize=10, loc="left", pad=4)

    # Panel 4 — binary crash/normal state derived from crash_events set
    ax4 = axes[3]
    crash_flag = [1 if i in set(crash_events) else 0
                  for i in range(len(accel_mag_sq))]
    ax4.fill_between(range(len(crash_flag)), crash_flag,
                     color=COLOR_BORDER, alpha=0.85, step="pre")
    ax4.set_yticks([0, 1])
    ax4.set_yticklabels(["Normal", "Crash"], fontsize=8)
    ax4.set_ylabel("State", fontsize=10)
    ax4.set_xlabel("Sample index", fontsize=10)
    ax4.set_ylim(-0.05, 1.2)
    ax4.grid(True, axis="x", linewidth=0.3, alpha=0.4)
    ax4.set_title("Crash state", fontsize=10, loc="left", pad=4)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "log.csv"
    main(path)
