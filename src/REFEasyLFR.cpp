#include "REFEasyLFR.h"

REFEasyLFR::REFEasyLFR(const SensorConfig& sc,
                                 const MotorConfig&  mc,
                                 const PIDConfig&    pc)
 : _sr(sc)
 , _md(mc)
 , _lc(pc)
{}

void REFEasyLFR::begin() {
  _sr.begin();
  _md.begin();
  _lc.begin();
}

void REFEasyLFR::runOnce() {
  uint16_t pos = _sr.readLine();
  _lc.followLine(pos, _md);
}
