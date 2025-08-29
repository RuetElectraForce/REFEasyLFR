#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>

//Declaring enum class to distinguish MUX/Direct reading
enum class SensorType {
  MUX,       // selectPins[4] + sigPin
  DIRECT     // sensorPins[numSensors]
};

/*This enum class selects the MCU type, primarily distinguishing Arduino and Pi Pico.
The purpose is to separate the 10us delay which is used for Pico to alleviate the 
crosstalk issue that occurs during sensor reading*/
enum class MCUType {
  MCU_ARDUINO, 
  PICO
};

struct SensorConfig {
  SensorType type;

  //for MUX
  uint8_t  selectPins[4];   // S3, S2, S1, S0
  uint8_t  sigPin;          // analog input pin

  //for DIRECT
  uint8_t* sensorPins = nullptr; //pointer to user array
                                 //size = numSensors

  //shared config for both types
  uint8_t  numSensors;      // e.g. 14
  bool     invertReadings;  // true if you want 1023 - raw
  uint16_t flagMinTh;       // noise threshold
  uint16_t flagMaxTh;       // "on‑line" threshold
  MCUType mcu = MCUType :: MCU_ARDUINO;//Microcontroller type is set default to Arduino.

  // optional helpers inside SensorConfig struct: (Factory Methods)
  static SensorConfig useMux(uint8_t s3, uint8_t s2, uint8_t s1, uint8_t s0,
                              uint8_t sig, uint8_t n,
                              bool inv=false,
                              uint16_t minT=0, uint16_t maxT=1023,
                              MCUType m = MCUType :: MCU_ARDUINO)
  {
    SensorConfig c;
    c.type          = SensorType::MUX;
    c.selectPins[0] = s3; c.selectPins[1] = s2;
    c.selectPins[2] = s1; c.selectPins[3] = s0;
    c.sigPin        = sig;
    c.numSensors    = n;
    c.invertReadings= inv;
    c.flagMinTh     = minT;
    c.flagMaxTh     = maxT;
    c.mcu           = m;
    return c;
  }

  static SensorConfig useDirect(uint8_t pins[], uint8_t n,
                                bool inv=false,
                                uint16_t minT=0, uint16_t maxT=1023,
                                MCUType m = MCUType :: MCU_ARDUINO)
  {
    SensorConfig c;
    c.type        = SensorType::DIRECT;
    c.sensorPins  = pins;
    c.numSensors  = n;
    c.invertReadings= inv;
    c.flagMinTh   = minT;
    c.flagMaxTh   = maxT;
    c.mcu         = m;
    return c;
  }
};

#define maxSensors 14
class SensorReader {
public:
  SensorReader(const SensorConfig& cfg);

  void begin();
  uint32_t readLine();                   // as given
  void calibrate(int iter = 10000);   // runs Calibration()
  void printCalibration();
  void printPosition();
  void saveCalibration(int initAddress = 0);
  void readCalibration(int initAddress = 0);
  void setInvert(bool invert);
  
  //__________New getters for accessiing _onLine and _sensor[] array______
  uint8_t getOnLine() const {return _onLine; }
  uint16_t getSensor(uint8_t idx) const {
    return (idx < _cfg.numSensors)
          ? _sensor[idx]
          : 0;
  } 

private:
  SensorConfig _cfg;
  
  uint16_t     _raw[maxSensors];
  int          _sensor[maxSensors];
  uint16_t     _minimum[maxSensors];
  uint16_t     _maximum[maxSensors];
  uint16_t     _mid[maxSensors];
  uint8_t      _onLine;
  uint16_t     _lastValue;
  static const byte _channelTable[maxSensors][4];

  void muxAddr(uint8_t idx);
};

#endif // SENSOR_READER_H

