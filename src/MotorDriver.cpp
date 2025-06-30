#include "MotorDriver.h"


MotorDriver::MotorDriver(const MotorConfig& cfg)
  : _cfg(cfg) {}

void MotorDriver::begin() {
  pinMode(_cfg.lfPin, OUTPUT);
  pinMode(_cfg.lbPin, OUTPUT);
  pinMode(_cfg.rfPin, OUTPUT);
  pinMode(_cfg.rbPin, OUTPUT);
  
  //PWM pins for only L298
  pinMode(_cfg.pwmL, OUTPUT);
  pinMode(_cfg.pwmR, OUTPUT);
}

//The user will call this function.
void MotorDriver::drive(int leftSpeed, int rightSpeed) {
  if (_cfg.type == DriverType::BTS) {
    _driveBTS(leftSpeed, rightSpeed);
  } else {
    _driveL298(leftSpeed, rightSpeed);
  }
}


//Depending on the driver type any one of the below function will run
void MotorDriver::_driveBTS(int l, int r) {
  if (l > 0) {
    analogWrite(_cfg.lfPin, l);
    analogWrite(_cfg.lbPin, 0);
  } else {
    l = -l;
    analogWrite(_cfg.lfPin, 0);
    analogWrite(_cfg.lbPin, l);
  }

  if (r > 0) {
    analogWrite(_cfg.rfPin, r);
    analogWrite(_cfg.rbPin, 0);
  } else {
    r = -r;
    analogWrite(_cfg.rfPin, 0);
    analogWrite(_cfg.rbPin, r);
  } 
}

void MotorDriver :: _driveL298(int l, int r){
  Serial.println("l298");
  if (l > 0) {
    digitalWrite(_cfg.lfPin, 1);
    digitalWrite(_cfg.lbPin, 0);
  } else {
    l = -l;
    digitalWrite(_cfg.lfPin, 0);
    digitalWrite(_cfg.lbPin, 1);
  }

  if (r > 0) {
    digitalWrite(_cfg.rfPin, 1);
    digitalWrite(_cfg.rbPin, 0);
  } else {
    r = -r;
    digitalWrite(_cfg.rfPin, 0);
    digitalWrite(_cfg.rbPin, 1);
  }
  analogWrite(_cfg.pwmL, l);
  analogWrite(_cfg.pwmR, r); 
}