#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const unsigned long Controller_c::ARM_STABLE_MS;
const unsigned long Controller_c::POST_STOP_SETTLE_MS;
const unsigned long Controller_c::TURN_TIMEOUT_MS;
const unsigned long Controller_c::BUTTON_LOCKOUT_MS;
const unsigned long Controller_c::LOG_INTERVAL_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS),
    log_timer(LOG_INTERVAL_MS) {
  reset();
}

void Controller_c::reset() {
  signal = 0;
  phase = WAITING;
  recovery_start_ms = 0;

  trial_index = 0;
  direction = 0;
  target_counts = 0.0f;
  turn_start_left = 0;
  turn_start_right = 0;
  arm_start_ms = 0;
  mode_start_ms = 0;
  turn_start_ms = 0;
  turn_stop_ms = 0;
  last_button_ms = -1000000L;
  last_arm_left = 0;
  last_arm_right = 0;

  imu_available = false;
  gyro_bias_dps = 0.0f;
  gyro_yaw_rad = 0.0f;
  gyro_yaw_at_stop_rad = 0.0f;
  gyro_last_dps = 0.0f;
  gyro_sum_dps = 0.0f;
  gyro_sum_count = 0;
  last_imu_timestamp_us = 0;
  imu_sample_count = 0;
}

uint8_t Controller_c::getSignal() const {
  return signal;
}

void Controller_c::setSignal(uint8_t requested_signal) {
  if (requested_signal == 0) {
    reset();
    return;
  }

  if (signal == 0) {
    signal = 1;
  }
}

float Controller_c::targetCountsFor(float degrees) const {
  float mean_radius = 0.5f * (odometry.leftWheelRadiusMm +
                              odometry.rightWheelRadiusMm);
  if (mean_radius <= 0.0f || odometry.wheelSeparationMm <= 0.0f ||
      odometry.encoderCountsPerRevolution <= 0.0f) {
    return 0.0f;
  }
  return degrees * (PI / 180.0f) * odometry.wheelSeparationMm *
         odometry.encoderCountsPerRevolution / (2.0f * PI * mean_radius);
}

void Controller_c::begin(bool imu_detected) {
  imu_available = imu_detected;
  target_counts = targetCountsFor(TURN_TARGET_DEG);

  Serial0.println();
  Serial0.println("CONFIG,version=turn_test_v2");
  Serial0.print("CONFIG,pwm=");
  Serial0.print(TURN_TEST_PWM, 1);
  Serial0.print(",target_deg=");
  Serial0.print(TURN_TARGET_DEG, 1);
  Serial0.print(",target_counts=");
  Serial0.print(target_counts, 2);
  Serial0.print(",alternate=");
  Serial0.print(TURN_TEST_ALTERNATE_DIRECTION ? 1 : 0);
  Serial0.print(",imu=");
  Serial0.print(imu_available ? 1 : 0);
  Serial0.print(",wheel_r_mm=");
  Serial0.print(odometry.leftWheelRadiusMm, 2);
  Serial0.print(",wheel_sep_mm=");
  Serial0.print(odometry.wheelSeparationMm, 2);
  Serial0.print(",counts_per_rev=");
  Serial0.println(odometry.encoderCountsPerRevolution, 2);
  if (!imu_available) {
    Serial0.println("WARNING,IMU not detected: gyro reference unavailable");
  }
  Serial0.println("time_ms,trial,phase,direction,left_count,right_count,"
                  "progress_counts,odom_x_mm,odom_y_mm,odom_theta_rad,"
                  "gyro_z_dps,gyro_yaw_rad");
  Serial0.println("READY,place robot at START facing the reference line,"
                  "then press the button once");
}

void Controller_c::pollIMU(RobotIMU_c &imu) {
  if (!imu_available) {
    return;
  }

  if (!imu.isSampleReady() || !imu.read()) {
    return;
  }

  imu_sample_count++;
  const RobotIMUSample_t &sample = imu.getSample();
  float gz = GYRO_Z_SIGN * RobotIMU_c::gyroscopeToDegreesPerSecond(
               sample.gyroscope[RobotIMU_c::Z_AXIS]);
  gyro_last_dps = gz;

  if (phase == ARMING) {
    // The robot should be stationary, so the mean rate is the current bias.
    gyro_sum_dps += gz;
    gyro_sum_count++;
  } else if (phase == TURNING || phase == SETTLING || phase == DONE) {
    if (last_imu_timestamp_us != 0 &&
        sample.timestamp_us > last_imu_timestamp_us) {
      uint32_t dt_us = sample.timestamp_us - last_imu_timestamp_us;
      if (dt_us <= 100000UL) {
        gyro_yaw_rad += (gz - gyro_bias_dps) * (PI / 180.0f) *
                        (float)dt_us * 1.0e-6f;
      }
    }
  }

  last_imu_timestamp_us = sample.timestamp_us;
}

void Controller_c::startArming(Robot_c &robot, unsigned long now) {
  trial_index++;
  if (TURN_TEST_ALTERNATE_DIRECTION) {
    direction = (trial_index % 2 == 1) ? 1 : -1;
  } else {
    direction = TURN_TEST_FIRST_DIRECTION;
  }
  target_counts = targetCountsFor(TURN_TARGET_DEG);

  phase = ARMING;
  arm_start_ms = now;
  mode_start_ms = now;
  gyro_sum_dps = 0.0f;
  gyro_sum_count = 0;
  gyro_yaw_rad = 0.0f;
  gyro_yaw_at_stop_rad = 0.0f;
  last_arm_left = robot.getLeftEncoderCount();
  last_arm_right = robot.getRightEncoderCount();
  turn_start_left = last_arm_left;
  turn_start_right = last_arm_right;

  // Blue for left, red for right, so the operator can confirm the setup.
  if (direction > 0) {
    robot.setLED(0, 0, 255, 40);
  } else {
    robot.setLED(255, 0, 0, 40);
  }

  Serial0.print("TRIAL_ARM,");
  Serial0.print(trial_index);
  Serial0.print(",");
  Serial0.print(direction > 0 ? "LEFT" : "RIGHT");
  Serial0.print(",imu=");
  Serial0.print(imu_available ? 1 : 0);
  Serial0.print(",samples=");
  Serial0.println((unsigned long)imu_sample_count);
}

void Controller_c::finishArming(Robot_c &robot, unsigned long now) {
  if (gyro_sum_count > 0) {
    gyro_bias_dps = gyro_sum_dps / (float)gyro_sum_count;
  }
  if (!imu_available) {
    gyro_bias_dps = 0.0f;
  }

  turn_start_left = robot.getLeftEncoderCount();
  turn_start_right = robot.getRightEncoderCount();
  odometry.reset(0.0f, 0.0f, 0.0f, turn_start_left, turn_start_right);

  gyro_yaw_rad = 0.0f;
  gyro_yaw_at_stop_rad = 0.0f;
  turn_start_ms = now;
  phase = TURNING;

  robot.setLED(0, 255, 0, 40);

  Serial0.print("TURN_START,");
  Serial0.print(trial_index);
  Serial0.print(",bias_dps=");
  Serial0.print(gyro_bias_dps, 4);
  Serial0.print(",target_counts=");
  Serial0.print(target_counts, 2);
  Serial0.print(",start_L=");
  Serial0.print(turn_start_left);
  Serial0.print(",start_R=");
  Serial0.print(turn_start_right);
  Serial0.print(",samples=");
  Serial0.println((unsigned long)imu_sample_count);
}

void Controller_c::stopTurn(Robot_c &robot, unsigned long now) {
  robot.setMotorPWM(0, 0);
  robot.setLED(0, 0, 0, 0);
  turn_stop_ms = now;
  gyro_yaw_at_stop_rad = gyro_yaw_rad;
  phase = SETTLING;

  int32_t progress = direction *
    ((robot.getRightEncoderCount() - turn_start_right) -
     (robot.getLeftEncoderCount() - turn_start_left));

  Serial0.print("TURN_STOP,");
  Serial0.print(trial_index);
  Serial0.print(",progress_counts=");
  Serial0.print((long)progress);
  Serial0.print(",odom_deg=");
  Serial0.print(odometry.pose.theta * (180.0f / PI), 2);
  Serial0.print(",gyro_deg=");
  Serial0.print(gyro_yaw_rad * (180.0f / PI), 2);
  Serial0.print(",turn_ms=");
  Serial0.println(now - turn_start_ms);
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server,
                          RobotIMU_c &imu) {
  (void)server;

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Poll the IMU on every call, before the 10 ms controller gate, so that
  // the 104 Hz gyro stream is not sampled down to 100 Hz.
  pollIMU(imu);

  if (!update_timer.isReady(now)) {
    return;
  }
  update_timer.resetTimer(now);

  // Refresh encoders; local odometry only advances after a successful read.
  if (robot.getEncoders()) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // One button press arms one trial. A DONE trial must be acknowledged with a
  // new press, and a held button cannot retrigger after the 250 ms debounce.
  bool button_ok = ((long)now - last_button_ms) > (long)BUTTON_LOCKOUT_MS;
  if ((phase == WAITING || phase == DONE) && button_ok &&
      (phase != DONE || now - mode_start_ms > 500) &&
      robot.isButtonPressed()) {
    last_button_ms = (long)now;
    startArming(robot, now);
  }

  switch (phase) {
    case ARMING: {
      int32_t left = robot.getLeftEncoderCount();
      int32_t right = robot.getRightEncoderCount();
      if (left != last_arm_left || right != last_arm_right) {
        // The robot was still being placed; restart the quiet window and the
        // bias average.
        last_arm_left = left;
        last_arm_right = right;
        arm_start_ms = now;
        gyro_sum_dps = 0.0f;
        gyro_sum_count = 0;
      } else if (now - arm_start_ms >= ARM_STABLE_MS) {
        finishArming(robot, now);
      }
      robot.setMotorPWM(0, 0);
      break;
    }

    case TURNING: {
      int32_t progress = direction *
        ((robot.getRightEncoderCount() - turn_start_right) -
         (robot.getLeftEncoderCount() - turn_start_left));
      if (progress >= target_counts ||
          now - turn_start_ms >= TURN_TIMEOUT_MS) {
        stopTurn(robot, now);
      } else {
        robot.setMotorPWM(
          -direction * TURN_TEST_PWM,
          direction * TURN_TEST_PWM
        );
      }
      break;
    }

    case SETTLING:
      robot.setMotorPWM(0, 0);
      if (now - turn_stop_ms >= POST_STOP_SETTLE_MS) {
        phase = DONE;
        mode_start_ms = now;
        reportTrialSummary(robot, now);
        Serial0.println("READY,reposition robot at a START line,"
                        "then press the button once");
      }
      break;

    case DONE:
    case WAITING:
    default:
      robot.setMotorPWM(0, 0);
      break;
  }

  if (log_timer.isReady(now)) {
    log_timer.resetTimer(now);
    reportSample(robot, now);
  }
}

void Controller_c::reportSample(Robot_c &robot, unsigned long now) {
  int32_t progress = 0;
  if (phase == TURNING || phase == SETTLING || phase == DONE) {
    progress = direction *
      ((robot.getRightEncoderCount() - turn_start_right) -
       (robot.getLeftEncoderCount() - turn_start_left));
  }

  Serial0.print(now);
  Serial0.print(",");
  Serial0.print(trial_index);
  Serial0.print(",");
  Serial0.print((int)phase);
  Serial0.print(",");
  Serial0.print((int)direction);
  Serial0.print(",");
  Serial0.print((long)robot.getLeftEncoderCount());
  Serial0.print(",");
  Serial0.print((long)robot.getRightEncoderCount());
  Serial0.print(",");
  Serial0.print((long)progress);
  Serial0.print(",");
  Serial0.print(odometry.pose.x, 2);
  Serial0.print(",");
  Serial0.print(odometry.pose.y, 2);
  Serial0.print(",");
  Serial0.print(odometry.pose.theta, 5);
  Serial0.print(",");
  Serial0.print(gyro_last_dps, 4);
  Serial0.print(",");
  Serial0.print(gyro_yaw_rad, 5);
  Serial0.print("\n");
}

void Controller_c::reportTrialSummary(Robot_c &robot, unsigned long now) {
  int32_t progress = direction *
    ((robot.getRightEncoderCount() - turn_start_right) -
     (robot.getLeftEncoderCount() - turn_start_left));
  float odom_deg = odometry.pose.theta * (180.0f / PI);
  float gyro_deg = gyro_yaw_rad * (180.0f / PI);
  float gyro_stop_deg = gyro_yaw_at_stop_rad * (180.0f / PI);

  Serial0.print("TRIAL_SUMMARY,");
  Serial0.print(trial_index);
  Serial0.print(",");
  Serial0.print(direction > 0 ? "LEFT" : "RIGHT");
  Serial0.print(",progress_counts=");
  Serial0.print((long)progress);
  Serial0.print(",odom_deg=");
  Serial0.print(odom_deg, 2);
  Serial0.print(",gyro_deg=");
  Serial0.print(gyro_deg, 2);
  Serial0.print(",gyro_at_stop_deg=");
  Serial0.print(gyro_stop_deg, 2);
  Serial0.print(",coast_deg=");
  Serial0.print(gyro_deg - gyro_stop_deg, 2);
  Serial0.print(",turn_ms=");
  Serial0.println(turn_stop_ms - turn_start_ms);
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server,
                                    unsigned long timestamp_ms) {
  server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
    timestamp_ms,
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],
    signal
  );
  if ( Serial ) { // If serial is connected
    Serial.printf(
      "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
      timestamp_ms,
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta,
      robot.getLeftMotorPWM(),
      robot.getRightMotorPWM(),
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      robot.surface.reading[0],
      robot.surface.reading[1],
      robot.surface.reading[2],
      robot.surface.reading[3],
      robot.surface.reading[4],
      signal
    );
  }
}
