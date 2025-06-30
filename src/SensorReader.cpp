#include "SensorReader.h"
#include <EEPROM.h>

const byte SensorReader::_channelTable[14][4] = {
  {0,0,0,0},{0,0,0,1},{0,0,1,0},{0,0,1,1},
  {0,1,0,0},{0,1,0,1},{0,1,1,0},{0,1,1,1},
  {1,0,0,0},{1,0,0,1},{1,0,1,0},{1,0,1,1},
  {1,1,0,0},{1,1,0,1},
};

SensorReader::SensorReader(const SensorConfig& cfg)
 : _cfg(cfg), _lastValue(0) {}

void SensorReader::begin() {
  if(_cfg.type == SensorType :: MUX) {
    //MUX: set select lines to output, signal pin to input
    for (uint8_t i = 0; i < 4; i++)
      pinMode(_cfg.selectPins[i], OUTPUT);
    pinMode(_cfg.sigPin, INPUT);
  }
  else {
    // DIRECT: set each analog pin to input
    for (uint8_t i = 0; i < _cfg.numSensors; i++) {
      pinMode(_cfg.sensorPins[i], INPUT);
    }
  }  
}

//Select the MUX channel
void SensorReader::muxAddr(uint8_t ch) {
  for (uint8_t b = 0; b < 4; b++)
    digitalWrite(_cfg.selectPins[3 - b],
                 _channelTable[ch][b]);
}

void SensorReader :: setInvert(bool invert) {
  _cfg.invertReadings = invert;
}

//Read the sensors and calculate weighted position
uint32_t SensorReader::readLine() {
  unsigned long sum = 0, weighted = 0;
  _onLine = 0;

  for(uint8_t i = 0; i <_cfg.numSensors; i++)
  {
    _raw[i] = 0;
  }

  for (uint8_t i = 0; i < _cfg.numSensors; i++) 
  {
    if(_cfg.type == SensorType :: MUX) { //MUX reading
      muxAddr(i);
      if (_cfg.mcu == MCUType::PICO) {   //If the MCU is PICO, this is essential to avoid crosstalking
        delayMicroseconds(10);
      }
      _raw[i] = analogRead(_cfg.sigPin);
    } else {                             //DIRECT reading
      _raw[i] = analogRead(_cfg.sensorPins[i]); /*not putting the delay condition here, 
                                                because if someone uses PICO, they must have to use a MUX */
    }

    //If the sensor detects white line on black surface
    if (_cfg.invertReadings) _raw[i] = 1023 - _raw[i];
  }

  for (uint8_t i = 0; i < _cfg.numSensors; i++) {
    // map between min/max → 0..1023
    _sensor[i] = map(_raw[i],
                     _minimum[i], _maximum[i],
                     -200, 1023);
    _sensor[i] = constrain(_sensor[i], 0, 1023);

    if (_sensor[i] > _cfg.flagMaxTh) _onLine++;
    
    if (_sensor[i] > _cfg.flagMinTh) {
      weighted += (unsigned long)_sensor[i] * i * 1000UL; //used unsigned long as UL to avoid overflow issues
      sum      += _sensor[i];
    }
  }

  if (_onLine == 0) {
    // no line: return edge
    if (_lastValue < (uint32_t)(_cfg.numSensors - 1) * 1000UL / 2)
      return 0;
    else
      return (uint32_t)(_cfg.numSensors - 1) * 1000UL;
  }

  _lastValue = weighted / sum;
  return _lastValue;
}

void SensorReader::calibrate() {
  // init
  for (uint8_t i = 0; i < _cfg.numSensors; i++) {
    _maximum[i] = 0;
    _minimum[i] = 1023;
  }

  pinMode(25, OUTPUT);
  digitalWrite(25, 1);
  // spin for N readings
  for (uint16_t t = 0; t < 10000; t++) {
    for (uint8_t i = 0; i < _cfg.numSensors; i++) {
      uint16_t v;
      if (_cfg.type == SensorType::MUX) {
        muxAddr(i);
        if (_cfg.mcu == MCUType::PICO) {
          delayMicroseconds(10);
        }
        v = analogRead(_cfg.sigPin);
      } else {
        v = analogRead(_cfg.sensorPins[i]);
      }
      _maximum[i] = max(_maximum[i], v);
      _minimum[i] = min(_minimum[i], v);
    }
  }
  digitalWrite(25, 0);
  printCalibration();
  saveCalibration(10);
  // mid‑points
  for (uint8_t i = 0; i < _cfg.numSensors; i++)
    _mid[i] = (_maximum[i] + _minimum[i]) / 2; 
}

void SensorReader::saveCalibration(int addr) {
  //Runs only on PICO cores
  #if defined(ARDUINO_ARCH_RP2040)
    EEPROM.begin(512);    //In PICO it is required to insert the memory size we want to use
    for (uint8_t i = 0; i < _cfg.numSensors; i++)
      EEPROM.write(addr + i, _minimum[i] / 4);
    for (uint8_t i = 0; i < _cfg.numSensors; i++)
      EEPROM.write(addr + 32 + i, _maximum[i] / 4);
    if (_cfg.mcu == MCUType::PICO) {
      EEPROM.commit();
      Serial.println("Saved to PICO");
    }
  #else
    EEPROM.begin();
    for (uint8_t i = 0; i < _cfg.numSensors; i++)
      EEPROM.write(addr + i, _minimum[i] / 4);
    for (uint8_t i = 0; i < _cfg.numSensors; i++)
      EEPROM.write(addr + 32 + i, _maximum[i] / 4);
      Serial.println("Saved Arduino");
  #endif
}

void SensorReader::readCalibration(int addr) {
  #if defined(ARDUINO_ARCH_RP2040)    //Reading portion for PICO
    EEPROM.begin(512);
    for (uint8_t i = 0; i < _cfg.numSensors; i++)
      _minimum[i] = EEPROM.read(addr + i) * 4;
    for (uint8_t i = 0; i < _cfg.numSensors; i++)
      _maximum[i] = EEPROM.read(addr + 32 + i) * 4;
  #else
    EEPROM.begin();   //Reading portion for Arduino
    for (uint8_t i = 0; i < _cfg.numSensors; i++)
      _minimum[i] = EEPROM.read(addr + i) * 4;
    for (uint8_t i = 0; i < _cfg.numSensors; i++)
      _maximum[i] = EEPROM.read(addr + 32 + i) * 4;
  #endif
  
  printCalibration();
}

void SensorReader::printCalibration() {
  for (uint8_t i = 0; i < _cfg.numSensors; i++) {
    Serial.print(_minimum[i]); Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < _cfg.numSensors; i++) {
    Serial.print(_maximum[i]); Serial.print(' ');
  }
  Serial.println();
}

void SensorReader :: printPosition()
{
  uint16_t pos = readLine();
  for (uint8_t i = 0; i < _cfg.numSensors; i++) {
    Serial.print(_sensor[i]);
    Serial.print("\t");
  }
  Serial.print("\t");
  Serial.print(pos);
  Serial.print("\t");
  Serial.println(_onLine);
}
