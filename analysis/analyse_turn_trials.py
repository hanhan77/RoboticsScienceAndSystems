#!/usr/bin/env python3
"""Analyse the 90-degree turn trials recorded by the StampC3 firmware.

The raw capture is the serial log written by MyFirstRobot/Controller.cpp.
Each CSV row has 12 fields:
    time_ms, trial, phase, direction, left_count, right_count,
    progress_counts, odom_x_mm, odom_y_mm, odom_theta_rad,
    gyro_z_dps, gyro_yaw_rad

Phases: 0 = waiting, 1 = turning, 2 = settling, 3 = done, 4 = arming.

The script computes, per trial:
  * encoder-odometry heading at the stop command and at the end of settling
  * gyro-integrated physical heading at those instants
  * direction-corrected heading error (negative = encoder under-reports)
  * effective wheel separation implied by the counts and the gyro heading

It writes a tidy per-trial CSV plus a text summary. No third-party packages
are required.
"""

from __future__ import annotations

import csv
import itertools
import math
import os
import sys
from statistics import mean, stdev

# ---------------------------------------------------------------------------
# Robot and experiment constants (must match Controller.h / OdometryModel.h)
# ---------------------------------------------------------------------------
WHEEL_RADIUS_MM = 16.0
COUNTS_PER_REV = 358.3
NOMINAL_SEPARATION_MM = 90.0
TARGET_DEG = 90.0
TARGET_COUNTS = (math.radians(TARGET_DEG) * NOMINAL_SEPARATION_MM *
                 COUNTS_PER_REV / (2.0 * math.pi * WHEEL_RADIUS_MM))
MM_PER_COUNT = 2.0 * math.pi * WHEEL_RADIUS_MM / COUNTS_PER_REV

# Trial 5 hit the firmware's 4 s safety timeout after a USB-cable snag.
EXCLUDED_TRIALS = {5}

# 95% two-sided t critical values for small samples.
T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447,
       7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179,
       13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101,
       19: 2.093, 20: 2.086}


def t_critical(n: int) -> float:
    """Return the two-sided 95% t critical value for n samples."""
    df = max(1, n - 1)
    return T95.get(df, 1.96)


def read_rows(path: str) -> list[list[float]]:
    rows = []
    with open(path, encoding="utf-8", errors="replace") as stream:
        for line in stream:
            parts = line.strip().split(",")
            if len(parts) != 12:
                continue
            try:
                rows.append([
                    int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]),
                    int(parts[4]), int(parts[5]), float(parts[6]),
                    float(parts[7]), float(parts[8]), float(parts[9]),
                    float(parts[10]), float(parts[11]),
                ])
            except ValueError:
                continue
    return rows


def trial_metrics(rows: list[list[float]], trial: int) -> dict:
    data = [r for r in rows if r[1] == trial]
    turning = [r for r in data if r[2] == 1]
    settling = [r for r in data if r[2] == 2]
    if not turning or not settling:
        raise ValueError(f"trial {trial}: missing turning or settling rows")

    start = turning[0]
    stop = settling[0]
    final = settling[-1]
    direction = start[3]
    if direction not in (-1, 1):
        raise ValueError(f"trial {trial}: bad direction {direction}")

    def progress(row: list[float]) -> int:
        return direction * ((row[5] - start[5]) - (row[4] - start[4]))

    progress_stop = progress(stop)
    progress_final = progress(final)
    gyro_stop = math.degrees(stop[11])
    gyro_final = math.degrees(final[11])
    odom_stop = math.degrees(stop[9])
    odom_final = math.degrees(final[9])
    duration_ms = stop[0] - start[0]

    def sep(progress_counts: int, gyro_deg: float) -> float:
        angle_rad = abs(math.radians(gyro_deg))
        if angle_rad < 1e-6:
            return float("nan")
        return progress_counts * MM_PER_COUNT / angle_rad

    return {
        "trial": trial,
        "direction": "L" if direction > 0 else "R",
        "direction_sign": direction,
        "duration_ms": duration_ms,
        "progress_stop": progress_stop,
        "progress_final": progress_final,
        "odom_stop_deg": odom_stop,
        "gyro_stop_deg": gyro_stop,
        "odom_final_deg": odom_final,
        "gyro_final_deg": gyro_final,
        "coast_deg": direction * (gyro_final - gyro_stop),
        "err_stop_dir_deg": direction * (odom_stop - gyro_stop),
        "err_final_dir_deg": direction * (odom_final - gyro_final),
        "sep_stop_mm": sep(progress_stop, gyro_stop),
        "sep_final_mm": sep(progress_final, gyro_final),
        "valid": (trial not in EXCLUDED_TRIALS and
                  progress_stop >= 0.8 * TARGET_COUNTS and
                  duration_ms < 3900),
    }


def describe(values: list[float]) -> dict:
    n = len(values)
    if n == 0:
        return {"n": 0}
    avg = mean(values)
    sd = stdev(values) if n > 1 else 0.0
    sem = sd / math.sqrt(n) if n > 1 else 0.0
    half = t_critical(n) * sem
    return {"n": n, "mean": avg, "sd": sd, "sem": sem,
            "ci_low": avg - half, "ci_high": avg + half}


def permutation_p(group_a: list[float], group_b: list[float]) -> float:
    """Two-sided permutation test on the difference in means (all splits)."""
    combined = list(group_a) + list(group_b)
    na = len(group_a)
    observed = abs(mean(group_a) - mean(group_b))
    count = 0
    total = 0
    for idx in itertools.combinations(range(len(combined)), na):
        chosen = set(idx)
        a = [combined[i] for i in range(len(combined)) if i in chosen]
        b = [combined[i] for i in range(len(combined)) if i not in chosen]
        total += 1
        if abs(mean(a) - mean(b)) >= observed - 1e-12:
            count += 1
    return count / total


def fmt(stat: dict, unit: str = "", digits: int = 2) -> str:
    if stat["n"] == 0:
        return "n=0"
    pd = f"{digits}"
    return (f"n={stat['n']}, mean={stat['mean']:.{pd}f}{unit}, "
            f"SD={stat['sd']:.{pd}f}{unit}, "
            f"95% CI [{stat['ci_low']:.{pd}f}, {stat['ci_high']:.{pd}f}]{unit}")


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    raw_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        root, "test_materials", "turn_capture_main.txt")
    out_dir = os.path.join(root, "results")
    os.makedirs(out_dir, exist_ok=True)

    rows = read_rows(raw_path)
    trial_ids = sorted({int(r[1]) for r in rows if r[1] > 0})
    metrics = []
    for trial in trial_ids:
        try:
            metrics.append(trial_metrics(rows, trial))
        except ValueError as exc:
            print(f"skip: {exc}")

    valid = [m for m in metrics if m["valid"]]
    left = [m for m in valid if m["direction_sign"] > 0]
    right = [m for m in valid if m["direction_sign"] < 0]

    # Calibrated separation from the commanded-turn segment (before coast).
    sep_cal = mean([m["sep_stop_mm"] for m in valid])

    # Corrected model: theta scales inversely with separation.
    for m in valid:
        scale = NOMINAL_SEPARATION_MM / sep_cal
        m["odom_stop_corrected_deg"] = m["odom_stop_deg"] * scale
        m["odom_final_corrected_deg"] = m["odom_final_deg"] * scale
        m["err_stop_corrected_deg"] = m["direction_sign"] * (
            m["odom_stop_corrected_deg"] - m["gyro_stop_deg"])
        m["err_final_corrected_deg"] = m["direction_sign"] * (
            m["odom_final_corrected_deg"] - m["gyro_final_deg"])

    columns = ["trial", "direction", "duration_ms", "progress_stop",
               "progress_final", "odom_stop_deg", "gyro_stop_deg",
               "odom_final_deg", "gyro_final_deg", "coast_deg",
               "err_stop_dir_deg", "err_final_dir_deg",
               "sep_stop_mm", "sep_final_mm", "valid",
               "odom_stop_corrected_deg", "odom_final_corrected_deg",
               "err_stop_corrected_deg", "err_final_corrected_deg"]
    csv_path = os.path.join(out_dir, "turn_trial_summary.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for m in metrics:
            writer.writerow(m)

    stats = {}
    for name, group in (("all", valid), ("left", left), ("right", right)):
        stats[name] = {
            "duration_ms": describe([m["duration_ms"] for m in group]),
            "err_stop": describe([m["err_stop_dir_deg"] for m in group]),
            "err_final": describe([m["err_final_dir_deg"] for m in group]),
            "sep_stop": describe([m["sep_stop_mm"] for m in group]),
            "sep_final": describe([m["sep_final_mm"] for m in group]),
            "coast": describe([m["coast_deg"] for m in group]),
            "err_stop_corrected": describe(
                [m["err_stop_corrected_deg"] for m in group]),
            "err_final_corrected": describe(
                [m["err_final_corrected_deg"] for m in group]),
        }

    # Error magnitude (under-report) rather than signed direction error.
    under_stop_left = [-m["err_stop_dir_deg"] for m in left]
    under_stop_right = [-m["err_stop_dir_deg"] for m in right]
    under_final_left = [-m["err_final_dir_deg"] for m in left]
    under_final_right = [-m["err_final_dir_deg"] for m in right]
    p_stop = permutation_p(under_stop_left, under_stop_right)
    p_final = permutation_p(under_final_left, under_final_right)
    p_sep = permutation_p([m["sep_stop_mm"] for m in left],
                          [m["sep_stop_mm"] for m in right])

    lines = []
    lines.append("TURN TRIAL ANALYSIS")
    lines.append("===================")
    lines.append(f"input: {raw_path}")
    lines.append(f"target counts for {TARGET_DEG:g} deg: {TARGET_COUNTS:.2f}")
    lines.append(f"nominal separation: {NOMINAL_SEPARATION_MM:g} mm")
    lines.append(f"mm per count: {MM_PER_COUNT:.6f} mm")
    lines.append("")
    lines.append("Per-trial data")
    lines.append("-" * 100)
    header = ("trial dir dur_ms prog_stop odom_stop gyro_stop odom_final "
              "gyro_final coast err_stop err_final sep_stop sep_final valid")
    lines.append(header)
    for m in metrics:
        lines.append(
            f"{m['trial']:5d} {m['direction']:>3s} {m['duration_ms']:6d} "
            f"{m['progress_stop']:9d} {m['odom_stop_deg']:9.2f} "
            f"{m['gyro_stop_deg']:9.2f} {m['odom_final_deg']:10.2f} "
            f"{m['gyro_final_deg']:10.2f} {m['coast_deg']:5.2f} "
            f"{m['err_stop_dir_deg']:8.2f} {m['err_final_dir_deg']:9.2f} "
            f"{m['sep_stop_mm']:8.2f} {m['sep_final_mm']:9.2f} "
            f"{str(m['valid']):>5s}")
    lines.append("")
    lines.append("Summary (direction-corrected error: negative = encoder "
                 "under-reports)")
    lines.append("-" * 100)
    for name in ("all", "left", "right"):
        s = stats[name]
        lines.append(f"[{name}]")
        lines.append(f"  duration        {fmt(s['duration_ms'], ' ms', 0)}")
        lines.append(f"  err at stop     {fmt(s['err_stop'], ' deg')}")
        lines.append(f"  err at settle   {fmt(s['err_final'], ' deg')}")
        lines.append(f"  coast            {fmt(s['coast'], ' deg')}")
        lines.append(f"  sep at stop     {fmt(s['sep_stop'], ' mm')}")
        lines.append(f"  sep at settle   {fmt(s['sep_final'], ' mm')}")
        lines.append(f"  corrected stop  {fmt(s['err_stop_corrected'], ' deg')}")
        lines.append(f"  corrected final {fmt(s['err_final_corrected'], ' deg')}")
    lines.append("")
    lines.append(f"calibrated separation (mean of stop estimates): "
                 f"{sep_cal:.3f} mm")
    lines.append(f"scale factor applied to odometry: "
                 f"{NOMINAL_SEPARATION_MM / sep_cal:.5f}")
    lines.append(f"separation vs nominal 90 mm: "
                 f"{(sep_cal - NOMINAL_SEPARATION_MM) / NOMINAL_SEPARATION_MM * 100:+.2f}%")
    lines.append("")
    lines.append("Left vs right (permutation test on the difference in means)")
    lines.append(f"  under-report at stop:  p = {p_stop:.3f}")
    lines.append(f"  under-report at settle: p = {p_final:.3f}")
    lines.append(f"  separation at stop:    p = {p_sep:.3f}")
    lines.append("")
    lines.append("Excluded trials: " + ", ".join(
        f"{m['trial']} ({m['duration_ms']} ms, {m['progress_stop']} counts)"
        for m in metrics if not m["valid"]))
    text = "\n".join(lines)
    with open(os.path.join(out_dir, "turn_summary_stats.txt"), "w",
              encoding="utf-8") as stream:
        stream.write(text + "\n")
    print(text)

    # Plot-friendly per-trial files (valid trials only).
    for label, group in (("left", left), ("right", right)):
        plot_path = os.path.join(out_dir, f"plot_{label}.csv")
        with open(plot_path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["trial", "sep_stop_mm", "under_stop_deg",
                             "under_final_deg", "coast_deg"])
            for m in group:
                writer.writerow([m["trial"], f"{m['sep_stop_mm']:.4f}",
                                 f"{-m['err_stop_dir_deg']:.4f}",
                                 f"{-m['err_final_dir_deg']:.4f}",
                                 f"{m['coast_deg']:.4f}"])

    # Time series for two representative trials: first left and first right.
    for label, group in (("left", left), ("right", right)):
        if not group:
            continue
        m = group[0]
        data = [r for r in rows if r[1] == m["trial"]]
        turning = [r for r in data if r[2] == 1]
        settling = [r for r in data if r[2] == 2]
        window = turning + settling
        t0 = window[0][0]
        series_path = os.path.join(out_dir, f"timeseries_trial_{m['trial']:02d}_{label}.csv")
        with open(series_path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "phase", "odom_deg", "gyro_deg",
                             "odom_mag", "gyro_mag"])
            for r in window:
                odom = math.degrees(r[9])
                gyro = math.degrees(r[11])
                writer.writerow([f"{(r[0] - t0) / 1000.0:.3f}", r[2],
                                 f"{odom:.4f}", f"{gyro:.4f}",
                                 f"{abs(odom):.4f}", f"{abs(gyro):.4f}"])


if __name__ == "__main__":
    main()
