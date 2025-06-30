#ifndef REF_EASY_LFR_H
#define REF_EASY_LFR_H

#include "SensorReader.h"
#include "MotorDriver.h"
#include "LineController.h"

class REFEasyLFR {
public:
  REFEasyLFR(const SensorConfig& sc,
                  const MotorConfig&  mc,
                  const PIDConfig&    pc);

  void begin();
  void runOnce();

  //Declaring the wrapper methods to access functions from other headers
  uint16_t readLine() {return _sr.readLine();} //need to write 'return' only if the method returns a value. 
  void calibrate() {_sr.calibrate();}          //for void methods no need to write return
  void readCalibration(int addr) {_sr.readCalibration(addr);}
  void printCalibration() {_sr.printCalibration();}
  void printPosition() {_sr.printPosition();}
  void drive(int l, int r) {_md.drive(l,r);}
  void printSpeed() {_lc.printSpeed();}
  void followLine(uint16_t position) {_lc.followLine(position, _md);}

  //Wrappers for accessing the _onLine and _sensor[] array
  uint8_t onLine() const {
    return _sr.getOnLine();
  }

  uint16_t sensorValue(uint8_t idx) const {
    return _sr.getSensor(idx);
  }

private:
  SensorReader   _sr;
  MotorDriver    _md;
  LineController _lc;
};

#endif // LINE_FOLLOWER_BOT_H
