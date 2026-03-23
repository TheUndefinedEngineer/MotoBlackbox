import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.patches as mpatches
import csv
import sys
import os
import math


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
                continue
    return samples


def insert_gaps(ts_sec, *data_lists, gap_threshold=2.0):
    """
    Insert NaN breaks wherever timestamps jump by more than gap_threshold seconds.
    This prevents matplotlib from drawing a straight line across missing data.
    Returns new ts_sec and data_lists with NaNs inserted at gap points.
    """
    out_ts   = []
    out_data = [[] for _ in data_lists]

    for i in range(len(ts_sec)):
        if i > 0 and (ts_sec[i] - ts_sec[i-1]) > gap_threshold:
            # Insert a NaN point at the midpoint of the gap
            mid = (ts_sec[i-1] + ts_sec[i]) / 2.0
            out_ts.append(mid)
            for j in range(len(data_lists)):
                out_data[j].append(float("nan"))

        out_ts.append(ts_sec[i])
        for j, d in enumerate(data_lists):
            out_data[j].append(d[i])

    return out_ts, out_data


def detect_crashes(accel_mag_sq, gyro_mag_sq,
                   accel_threshold, gyro_threshold,
                   consecutive_required=3):
    """
    Mirrors firmware logic exactly:
        if (accel_mag >= ACCEL_THRESHOLD || gyro_mag >= GYRO_THRESHOLD)
            crash_counter++
            if (crash_counter >= consecutive_required) → crash confirmed
        else
            crash_counter = 0
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
            crash_counter = 0
        counter_trace.append(crash_counter)

    return crash_events, counter_trace


def find_crash_windows(crash_events, timestamps, gap_ms=500):
    """
    Merge nearby confirmed-crash samples into windows.
    Uses actual timestamp difference instead of sample count
    so gaps in data don't accidentally merge separate events.
    """
    if not crash_events:
        return []
    windows = []
    start = crash_events[0]
    end   = crash_events[0]
    for idx in crash_events[1:]:
        if (timestamps[idx] - timestamps[end]) <= gap_ms:
            end = idx
        else:
            windows.append((start, end))
            start = idx
            end   = idx
    windows.append((start, end))
    return windows


def format_mmss(seconds):
    m = int(seconds) // 60
    s = int(seconds) % 60
    return f"{m:02d}:{s:02d}"


def main(csv_path):
    if not os.path.exists(csv_path):
        print(f"Error: file not found - {csv_path}")
        sys.exit(1)

    samples = read_csv(csv_path)
    if not samples:
        print("No valid samples found in CSV.")
        sys.exit(1)

    # ── Firmware thresholds ───────────────────────────────────────────────
    ACCEL_THRESHOLD    = 15_000_000
    GYRO_THRESHOLD     =  3_000_000
    CONSECUTIVE_NEEDED = 3            # 3 × 25 ms = 75 ms

    accel_mag_sq = []
    gyro_mag_sq  = []
    timestamps   = []

    for ax, ay, az, gx, gy, gz, ts in samples:
        if ax == ay == az == gx == gy == gz == 0:
            continue
        accel_mag_sq.append(ax*ax + ay*ay + az*az)
        gyro_mag_sq.append(gx*gx + gy*gy + gz*gz)
        timestamps.append(ts)

    crash_events, counter_trace = detect_crashes(
        accel_mag_sq, gyro_mag_sq,
        ACCEL_THRESHOLD, GYRO_THRESHOLD,
        CONSECUTIVE_NEEDED
    )

    crash_windows = find_crash_windows(crash_events, timestamps, gap_ms=500)

    # Time relative to first sample, in seconds
    t0     = timestamps[0]
    ts_sec = [(t - t0) / 1000.0 for t in timestamps]

    # Detect gap threshold: if average sample interval × 10
    avg_interval_s = (ts_sec[-1] - ts_sec[0]) / max(len(ts_sec) - 1, 1)
    gap_threshold  = max(avg_interval_s * 10, 1.0)   # at least 1 second

    # Insert NaN breaks at gaps so matplotlib doesn't interpolate across them
    ts_plot, (a_plot, g_plot, c_plot) = insert_gaps(
        ts_sec, accel_mag_sq, gyro_mag_sq, counter_trace,
        gap_threshold=gap_threshold
    )

    # ── Console summary ───────────────────────────────────────────────────
    print(f"Total samples         : {len(accel_mag_sq)}")
    print(f"Recording start       : {t0:.0f} ms")
    print(f"Recording end         : {timestamps[-1]:.0f} ms")
    print(f"Total span            : {(timestamps[-1]-t0)/1000:.1f} s")
    print(f"Crash events detected : {len(crash_windows)}")
    for i, (s, e) in enumerate(crash_windows, 1):
        t_s    = (timestamps[s] - t0) / 1000.0
        t_e    = (timestamps[e] - t0) / 1000.0
        peak_a = max(accel_mag_sq[s:e+1])
        peak_g = max(gyro_mag_sq[s:e+1])
        print(f"  Event {i}: {format_mmss(t_s)} → {format_mmss(t_e)} "
              f"| peak accel_sq {peak_a:>12,.0f} "
              f"| peak gyro_sq {peak_g:>11,.0f}")
    if not crash_windows:
        print("No crash detected in this log.")

    # ── Plot ─────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(15, 11), sharex=True,
                             gridspec_kw={"height_ratios": [2, 2, 1, 1]})
    fig.suptitle(
        f"IMU Crash Detection  |  trigger = {CONSECUTIVE_NEEDED} consecutive samples "
        f"(≈{CONSECUTIVE_NEEDED*25} ms)  |  {len(crash_windows)} event(s) detected",
        fontsize=12, fontweight="bold", y=0.99)

    COLOR_ACCEL  = "#378ADD"
    COLOR_GYRO   = "#1D9E75"
    COLOR_THRESH = "#E24B4A"
    COLOR_FILL   = "#FAEEDA"
    COLOR_BORDER = "#E24B4A"
    COLOR_COUNT  = "#BA7517"

    def shade_crashes(ax):
        for i, (s, e) in enumerate(crash_windows):
            xs = (timestamps[s] - t0) / 1000.0
            xe = (timestamps[e] - t0) / 1000.0
            # Give single-sample events a minimum visible width of 0.5 s
            if xe - xs < 0.5:
                xe = xs + 0.5
            ax.axvspan(xs, xe, color=COLOR_FILL, alpha=0.7,
                       label="Crash window" if i == 0 else "")
            ax.axvline(x=xs, color=COLOR_BORDER, linewidth=1.5,
                       linestyle="--", label="Crash start" if i == 0 else "")

    # Panel 1 — Accel squared
    ax1 = axes[0]
    ax1.plot(ts_plot, a_plot, color=COLOR_ACCEL, linewidth=0.7, label="Accel mag²")
    ax1.axhline(ACCEL_THRESHOLD, color=COLOR_THRESH, linestyle=":",
                linewidth=1.2, label=f"Threshold ({ACCEL_THRESHOLD:,})")
    shade_crashes(ax1)
    ax1.set_ylabel("Accel mag²", fontsize=10)
    ax1.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax1.grid(True, linewidth=0.3, alpha=0.4)
    ax1.set_title("Accelerometer (squared)", fontsize=10, loc="left", pad=4)

    # Panel 2 — Gyro squared
    ax2 = axes[1]
    ax2.plot(ts_plot, g_plot, color=COLOR_GYRO, linewidth=0.7, label="Gyro mag²")
    ax2.axhline(GYRO_THRESHOLD, color=COLOR_THRESH, linestyle=":",
                linewidth=1.2, label=f"Threshold ({GYRO_THRESHOLD:,})")
    shade_crashes(ax2)
    ax2.set_ylabel("Gyro mag²", fontsize=10)
    ax2.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax2.grid(True, linewidth=0.3, alpha=0.4)
    ax2.set_title("Gyroscope (squared)", fontsize=10, loc="left", pad=4)

    # Panel 3 — crash_counter
    ax3 = axes[2]
    ax3.plot(ts_plot, c_plot, color=COLOR_COUNT, linewidth=0.8, label="crash_counter")
    ax3.axhline(CONSECUTIVE_NEEDED, color=COLOR_THRESH, linestyle=":",
                linewidth=1.2, label=f"Trigger level ({CONSECUTIVE_NEEDED})")
    ax3.set_ylabel("Counter", fontsize=10)
    ax3.set_yticks(range(0, CONSECUTIVE_NEEDED + 2))
    ax3.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax3.grid(True, linewidth=0.3, alpha=0.4)
    ax3.set_title(f"Firmware crash_counter", fontsize=10, loc="left", pad=4)

    # Panel 4 — Crash state using axvspan per window (not fill_between)
    # fill_between fails for isolated samples — axvspan always draws a visible block
    ax4 = axes[3]
    ax4.set_facecolor("white")
    for i, (s, e) in enumerate(crash_windows):
        xs = (timestamps[s] - t0) / 1000.0
        xe = (timestamps[e] - t0) / 1000.0
        if xe - xs < 0.5:
            xe = xs + 0.5
        ax4.axvspan(xs, xe, ymin=0.1, ymax=0.9,
                    color=COLOR_BORDER, alpha=0.85,
                    label="Crash" if i == 0 else "")

    ax4.set_yticks([0, 1])
    ax4.set_yticklabels(["Normal", "Crash"], fontsize=8)
    ax4.set_ylim(-0.05, 1.2)
    ax4.set_ylabel("State", fontsize=10)
    ax4.set_xlabel("Time from recording start", fontsize=10)
    ax4.grid(True, axis="x", linewidth=0.3, alpha=0.4)
    ax4.set_title("Crash state", fontsize=10, loc="left", pad=4)
    if crash_windows:
        ax4.legend(loc="upper right", fontsize=8, framealpha=0.9)

    # x-axis formatted as mm:ss
    def sec_to_mmss(x, pos):
        if x < 0:
            return ""
        m = int(x) // 60
        s = int(x) % 60
        return f"{m:02d}:{s:02d}"

    total_span = ts_sec[-1]
    major_step = 30 if total_span < 600 else 60
    minor_step = 10 if total_span < 600 else 30

    for ax in axes:
        ax.xaxis.set_major_formatter(ticker.FuncFormatter(sec_to_mmss))
        ax.xaxis.set_major_locator(ticker.MultipleLocator(major_step))
        ax.xaxis.set_minor_locator(ticker.MultipleLocator(minor_step))

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "log.csv"
    main(path)
