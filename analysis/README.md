# 90° turn odometry experiment — analysis

This folder contains the processing code for the individual odometry
investigation. The raw data are the serial log produced by
`MyFirstRobot/Controller.cpp` and saved as
`test_materials/turn_capture_main.txt`.

## Run

```bash
cd /Users/han/file/robotics/MyFirstRobot
python3 analysis/analyse_turn_trials.py
```

No third-party Python packages are required.

## Input and outputs

- Input: `test_materials/turn_capture_main.txt`
  - 12 CSV columns: `time_ms, trial, phase, direction, left_count,
    right_count, progress_counts, odom_x_mm, odom_y_mm, odom_theta_rad,
    gyro_z_dps, gyro_yaw_rad`
  - phases: 0 waiting, 1 turning, 2 settling, 3 done, 4 arming
- `results/turn_trial_summary.csv` — per-trial metrics, including the
  stop-time and settle-time headings, coast, errors, effective separation,
  and validity flag
- `results/turn_summary_stats.txt` — means, SDs, 95% confidence intervals,
  calibrated separation, and left/right permutation tests
- `results/plot_left.csv`, `results/plot_right.csv` — data used by
  `SEMTM0043_42_IndividualTechReport/figures/fig_summary.pdf`
- `results/timeseries_trial_01_left.csv`,
  `results/timeseries_trial_02_right.csv` — data used by
  `SEMTM0043_42_IndividualTechReport/figures/fig_timeseries.pdf`

## Protocol

1. Five left and five right trials, alternating.
2. Each trial: 500 ms stationary arming window, odometry reset at the
   captured count baseline, turn at ±35 PWM until the differential count
   reaches 503.86 counts (nominal 90°), then 1.5 s settling.
3. Independent heading reference: LSM6DS33 gyro Z integrated at 104 Hz;
   zero bias re-estimated in each 500 ms arming window.
4. Test surface: one A4 placement sheet taped to a desk; robot centred on
   the cross with the same USB-cable routing.

## Excluded trial

Trial 5 is excluded. The USB cable snagged, the robot reached only
85 differential counts (16.5°) before the firmware's 4 s safety timeout.
The analysis also flags it automatically because `progress_stop` is below
80% of the target and the duration is at the timeout.

## Key results (10 valid trials)

- Encoder heading at the stop: 90.52° ± 0.25°
- Gyro heading at the stop: 94.77° ± 1.05°
- Encoder under-report: 4.25° ± 1.09° at the stop; 5.17° ± 1.75° after
  settling
- Effective wheel separation: 85.98 ± 0.99 mm (4.47% below 90 mm)
- Rescaled odometry (90/85.98) stop residual: −0.01° ± 1.09°
- Left/right difference: not significant (permutation p = 0.135 at the
  stop; p = 0.437 after settling)

## Remaining check

The gyro's absolute scale is taken from the LSM6DS33 datasheet
(8.75 mdps/LSB). `test_materials/angle_reference_sheet.pdf` is an
independent printed angle reference. Print it at 100%, tape it flat, run one
turn with the robot centred on the cross facing 0°, and read the final
heading from the arc. Compare that reading with the gyro heading printed in
the trial summary. If they differ by more than about 2°, revise the gyro
scale before interpreting the results.
