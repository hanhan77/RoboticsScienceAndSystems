# Individual odometry test materials

- `turn_capture_main.txt` — raw serial capture for the final experiment:
  trials 1–11 (5 left + 5 right valid; trial 5 rejected after a USB-cable
  snag caused the 4 s safety timeout). This is the original data for the
  report.
- `turn_capture_v2_validation.txt` — pilot capture (first successful
  left-turn trial), used only for firmware validation and not in the final
  statistics.
- `alignment_cross_sheet.pdf` — A4 placement sheet used for all ten valid
  trials. Print at 100% A4, tape flat; place the robot centred on the cross
  facing the arrow. The line is a placement reference only; the firmware
  does not read the surface sensors.
- `angle_reference_sheet.pdf` — A4 printed angle reference for an
  independent check of the gyro scale. Print at 100%, tape flat, place the
  robot centred on the cross facing 0°, run one turn, then read the final
  heading from the arc (5° ticks, 30° labels).
- `odometry_90deg_turn_test.pdf` — earlier L-shaped line-following map. It
  is not used for the in-place turn experiment, but it is the sheet that
  motivated the original 90° test.
- `turn_test_template.csv` — template from the earlier fixed-duration
  protocol; the final capture uses the 12-column format documented in
  `analysis/README.md`.

## Firmware parameters

Edit `MyFirstRobot/Controller.h`:

```cpp
TURN_TEST_PWM              = 35
TURN_TEST_ALTERNATE_DIRECTION = true
TURN_TARGET_DEG            = 90
ARM_STABLE_MS              = 500
POST_STOP_SETTLE_MS        = 1500
TURN_TIMEOUT_MS            = 4000
```

The direction alternates on successive trials: trial 1 left (blue LED),
trial 2 right (red LED), and so on.

## Reproducing the report figures

```bash
cd /Users/han/file/robotics/MyFirstRobot

# 1. Process the raw capture
python3 analysis/analyse_turn_trials.py

# 2. Build the two vector figures
cd SEMTM0043_42_IndividualTechReport/figures
tectonic fig_timeseries.tex
tectonic fig_summary.tex

# 3. Build the one-page report
cd ..
tectonic main.tex
```

`main.tex` currently contains `[YOUR 7-DIGIT ID]`; replace it with the
student's Bristol student ID before submission.
