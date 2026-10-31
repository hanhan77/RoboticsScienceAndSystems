#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotIMU.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/**
 * @brief Runs the encoder-targeted 90-degree turn experiment.
 *
 * Controller_c owns the experiment state and the local odometry estimate.
 * Robot_c owns the latest hardware snapshot and RobotIMU_c supplies the
 * independent gyro heading reference. The line-following constants and legacy
 * helpers below are retained for the group controller but are not called by
 * the experiment path; edit the TURN_TEST_* constants in Controller.h.
 */
class Controller_c {
  
  public:
    /** @brief Editable local encoder-to-pose model; inspect odometry.pose.
     * @note Controller mode resets do not reset this estimate. Use
     * odometry.reset(...) explicitly to start a new physical trial.
     */
    OdometryModel_c odometry;

    /** @brief Fastest interval between controller and I2C acquisition cycles. */
    static const unsigned long UPDATE_INTERVAL_MS = 10;
    /** @brief Interval between telemetry records in milliseconds. */
    static const unsigned long TELEMETRY_INTERVAL_MS = 50;
    /** @brief Maximum line-search duration before the controller stops. */
    static const unsigned long RECOVERY_TIMEOUT_MS = 5000;
    /** @brief Surface-reading threshold used to decide whether a line is present. */
    static const uint16_t LINE_THRESHOLD = 1400;
    /** @brief Nominal forward PWM bias for line following. */
    static constexpr float BASE_PWM = 45.0f;
    /** @brief Proportional gain applied to the DN2-minus-DN4 surface error. */
    static constexpr float LINE_FOLLOW_GAIN = 0.01f;
    /** @brief Left PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_LEFT_PWM = -35.0f;
    /** @brief Right PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_RIGHT_PWM = 35.0f;
    /** @brief PWM magnitude for the controlled turn experiment. */
    static constexpr float TURN_TEST_PWM = 35.0f;
    /** @brief Direction used for the first trial and when alternation is off. */
    static constexpr int8_t TURN_TEST_FIRST_DIRECTION = 1;
    /** @brief Alternate left/right on successive trials to cancel battery drift. */
    static constexpr bool TURN_TEST_ALTERNATE_DIRECTION = true;
    /** @brief Encoder-target heading magnitude for every trial, in degrees. */
    static constexpr float TURN_TARGET_DEG = 90.0f;
    /** @brief Sign that makes gyro Z positive for an anticlockwise (left) turn. */
    static constexpr int8_t GYRO_Z_SIGN = 1;
    /** @brief Quiet period required before a trial starts, in milliseconds. */
    static const unsigned long ARM_STABLE_MS = 500;
    /** @brief Time allowed after the stop command for coasting, in milliseconds. */
    static const unsigned long POST_STOP_SETTLE_MS = 1500;
    /** @brief Safety stop if the encoder target is not reached, in milliseconds. */
    static const unsigned long TURN_TIMEOUT_MS = 4000;
    /** @brief Ignore repeat button events from one held press, in milliseconds. */
    static const unsigned long BUTTON_LOCKOUT_MS = 700;
    /** @brief Interval between CSV records in milliseconds. */
    static const unsigned long LOG_INTERVAL_MS = 20;

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    /** @brief Clears signal and mode state, returning the controller to waiting.
     * @note This does not reset the task timers or the local odometry estimate.
     * It sends no motor command: returning to waiting does not stop the motors.
     * Send robot.setMotorPWM(0, 0) separately and check the command result.
     */
    void reset();

    /**
     * @brief Returns whether the controller has received its start signal.
     * @return Zero while waiting; one after the button or caller starts it.
     */
    uint8_t getSignal() const;

    /**
     * @brief Starts the controller or resets it to waiting.
     * @param requested_signal Zero resets to waiting without sending a motor stop;
     * a non-zero value starts line following only if the current signal is zero.
     * @note The signal remains one after a line-search timeout. Reset to zero
     * before requesting another start; a repeated non-zero value has no effect.
     */
    void setSignal(uint8_t requested_signal);

    /**
     * @brief Prints the experiment configuration and CSV header.
     * @param imu_available Result of RobotIMU_c::initialise().
     */
    void begin(bool imu_available);

    /**
     * @brief Runs one acquisition and experiment-control cycle.
     * @param robot Hardware-facing interface and cached measurement snapshot.
     * @param server Optional WiFi/TCP telemetry transport (unused here).
     * @param imu Direct inertial-sensor interface used as heading reference.
     * @note The IMU is polled on every call, before the 10 ms controller gate.
     * Each accepted 10 ms cycle refreshes the encoders and updates local
     * odometry. A 20 ms timer emits one high-precision CSV record.
     */
    void update(Robot_c &robot, RobotWifiAP_c &server, RobotIMU_c &imu);

  private:
    /** @brief Experiment phases written to the CSV "phase" column. */
    enum Phase {
      WAITING = 0,
      TURNING = 1,
      SETTLING = 2,
      DONE = 3,
      ARMING = 4
    };

    uint8_t signal;
    Phase phase;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;
    TaskTimer_c log_timer;
    unsigned long recovery_start_ms;

    uint8_t trial_index;
    int8_t direction;
    float target_counts;
    int32_t turn_start_left;
    int32_t turn_start_right;
    unsigned long arm_start_ms;
    unsigned long mode_start_ms;
    unsigned long turn_start_ms;
    unsigned long turn_stop_ms;
    long last_button_ms;
    int32_t last_arm_left;
    int32_t last_arm_right;

    bool imu_available;
    float gyro_bias_dps;
    float gyro_yaw_rad;
    float gyro_yaw_at_stop_rad;
    float gyro_last_dps;
    float gyro_sum_dps;
    uint32_t gyro_sum_count;
    uint32_t last_imu_timestamp_us;
    uint32_t imu_sample_count;

    /** @brief Reads each ready IMU sample and integrates gyro Z into heading. */
    void pollIMU(RobotIMU_c &imu);

    /** @brief Starts the quiet arming window for one trial. */
    void startArming(Robot_c &robot, unsigned long now);

    /** @brief Captures the count baseline, fixes gyro bias, and starts the turn. */
    void finishArming(Robot_c &robot, unsigned long now);

    /** @brief Stops both motors and enters the post-turn settling window. */
    void stopTurn(Robot_c &robot, unsigned long now);

    /** @brief Returns the differential encoder count for a target heading. */
    float targetCountsFor(float degrees) const;

    /** @brief Reports one high-precision CSV record. */
    void reportSample(Robot_c &robot, unsigned long now);

    /** @brief Reports the completed trial's heading estimates over Serial. */
    void reportTrialSummary(Robot_c &robot, unsigned long now);

    /**
     * @brief Sends the current controller and robot snapshot as one CSV record.
     * @note Controller.cpp writes these 14 columns, in order: timestamp_ms,
     * x, y, theta, left_pwm, right_pwm, left_count, right_count,
     * DN1, DN2, DN3, DN4, DN5, signal. These labels are not sent as a header.
     * x and y are in mm and theta is in radians, from the local Euler estimate,
     * not robot.pose from I2C. PWM values are cached requests, not measured output.
     * The timestamp is the controller-cycle start time; it does not guarantee
     * that every cached reading was successfully refreshed in that cycle.
     */
    void publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);
};

#endif
