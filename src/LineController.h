#ifndef LINE_CONTROLLER_H
#define LINE_CONTROLLER_H

#include <Arduino.h>
#include"MotorDriver.h"

struct PIDConfig {
  float KpMin, KpMax, Ki, KdMin, KdMax;
  int   baseSpeed, minSpeed, maxSpeed;
  // float divisor;      // for dynamic speed scaling
  uint16_t ref;
};

class LineController {
public:
  LineController(const PIDConfig& cfg);

  void begin();
  void followLine(uint16_t position, MotorDriver& md);
  void printSpeed();

private:
  PIDConfig _cfg;
  long       _lastError;
  float      _integral;
};

#endif // LINE_CONTROLLER_H


