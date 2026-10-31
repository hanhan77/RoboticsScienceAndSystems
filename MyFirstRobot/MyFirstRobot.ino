#include <Wire.h>

#include "RobotWifiAP.h"

/** @brief WiFi access-point and TCP server used for optional telemetry. */
RobotWifiAP_c server("iiiao", "myPassword", 80);

#include "Robot.h"
#include "RobotIMU.h"
#include "Controller.h"


/** @brief Hardware-facing interface to the Pololu robot. */
Robot_c robot;
/** @brief Direct interface to the 3Pi+ accelerometer, gyro, and magnetometer. */
RobotIMU_c imu;
/** @brief Supplied nominal line-following controller. */
Controller_c controller;


/** @brief Initialises serial, the robot interface, IMU, and the turn experiment. */
void setup() {

  Serial0.begin(115200);

  // Odometry exercises do not require the middleware team-name transaction.
  robot.initialise();
  delay(100);

  // The IMU shares the I2C bus owned by Robot_c, so it is configured after
  // robot.initialise(). Retry to survive a warm reset while the 32U4 is still
  // starting, then confirm that samples are actually arriving.
  bool imu_init_ok = false;
  for (uint8_t attempt = 0; attempt < 3 && !imu_init_ok; attempt++) {
    imu_init_ok = imu.initialise();
    if (!imu_init_ok) {
      delay(200);
    }
  }

  bool imu_streaming = false;
  if (imu_init_ok) {
    unsigned long wait_start = millis();
    while (millis() - wait_start < 500) {
      if (imu.isSampleReady() && imu.read()) {
        imu_streaming = true;
        break;
      }
      delay(2);
    }
  }

  Serial0.print("IMU,init=");
  Serial0.print(imu_init_ok ? 1 : 0);
  Serial0.print(",streaming=");
  Serial0.print(imu_streaming ? 1 : 0);
  Serial0.print(",available=");
  Serial0.println(imu.isAvailable() ? 1 : 0);

  controller.begin(imu_streaming);

  robot.setMotorPWM(0, 0);
}


/** @brief Runs one turn-experiment cycle. */
void loop() {
  // Experiment logic lives in Controller.cpp; edit parameters in Controller.h.
  controller.update(robot, server, imu);

}
