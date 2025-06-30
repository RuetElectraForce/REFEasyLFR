#include "LineController.h"
#include "MotorDriver.h"

LineController::LineController(const PIDConfig& cfg)
  : _cfg(cfg), _lastError(0), _integral(0) {}

void LineController::begin() {
  // nothing for now
}
int baseSpeedM, lm, rm;
float Kp, Kd;
void LineController::followLine(uint16_t position, MotorDriver& md) {
  
  int error = _cfg.ref - position;  
  // Adjust PID gains dynamically with respect to error
  float Ap = _cfg.KpMax - _cfg.KpMin;
  float Ad = _cfg.KdMax - _cfg.KdMin;
  Kp = _cfg.KpMin + Ap * abs(error) / (float)_cfg.ref;
  Kd = _cfg.KdMin + Ad * abs(error) / (float)_cfg.ref;
  
  //Reduce the base speed based on the curvature which results more differences in lm and rm
  // float DS = abs(lm-rm);
  // float gain = (1-(DS/_cfg.divisor));
  // baseSpeedM = gain * (float)_cfg.baseSpeed;

  // integral term (simple)
  _integral = ((2.0/3.0) * _integral) + error - _lastError;
  
  float adjust = Kp * error
                + Kd * (error - _lastError)
                + _cfg.Ki * _integral;
  
  lm = _cfg.baseSpeed + adjust;
  rm = _cfg.baseSpeed - adjust;
  
  // constrain
  lm = constrain(lm, _cfg.minSpeed,  _cfg.maxSpeed);
  rm = constrain(rm, _cfg.minSpeed,  _cfg.maxSpeed);
  
  
  md.drive(lm, rm);
  _lastError = error;
}

void LineController :: printSpeed() 
{
  Serial.print(_cfg.baseSpeed);
  Serial.print("\t");
  Serial.print(baseSpeedM);
  Serial.print("\t");
  Serial.print(lm);
  Serial.print("\t");
  Serial.print(rm);
  Serial.print("\t");
  Serial.print(Kp);
  Serial.print("\t");
  Serial.println(Kd);
  
}