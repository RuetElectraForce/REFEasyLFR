#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

enum class DriverType { BTS, L298};

struct MotorConfig {
  DriverType type;

  //always needed
  uint8_t lfPin, lbPin;  // left forward/backward
  uint8_t rfPin, rbPin;  // right forward/backward

  //only needed when it is L298, setting default values for unused state to avoid errors
  uint8_t pwmL;
  uint8_t pwmR;
};

class MotorDriver {
public:
  MotorDriver(const MotorConfig& cfg);

  void begin();
  void drive(int leftSpeed, int rightSpeed);

private:
  MotorConfig _cfg;

  // Helpers for each driver type
  void _driveBTS(int l, int r);
  void _driveL298(int l, int r);
};

#endif // MOTOR_DRIVER_H
