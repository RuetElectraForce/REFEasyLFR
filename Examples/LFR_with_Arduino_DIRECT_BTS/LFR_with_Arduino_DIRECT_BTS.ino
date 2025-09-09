#include <REFEasyLFR.h>

//_________Motor pins__________
//Make sure to check the PWM capability of the pins of your corresponding Microcontroller
#define LF  6  // Left forward pin 
#define LB  9  // Left backward pin
#define RF  10 // Right forward pin
#define RB  11 // Right backward pin

//__________IR reading variables_________
#define numSensor 8

//_________PID variables__________
/*Here some dummy values are provided. 
Your values will obviously differ from these but will be comparable,
i.e.: The Kp_min/Kp_max will not be 10, 20, 100 or vice versa.*/

float Kp_min = 0.02, Kp_max = 0.08, Kd_min = 4.0, Kd_max = 8.0, Ki = 0.001;
int base_speed = 100, min_speed = -40, max_speed = 180;
uint16_t reference = 6500;

//_________Setup the 3 configuration structures__________
// Direct reading Upto eight analog pins A0–A7 (Upto 16 analog pins if ATMega2560 is used)
const uint8_t pins[numSensor] = {A0,A1,A2,A3,A4,A5,A6,A7};

SensorConfig sc = SensorConfig::useDirect(
/* All input pins */pins,
/* numSensors     */ numSensor,
/* flagMinTh      */false,
/* flagMinTh      */ 50,
/* flagMaxTh      */ 200,
/* mcuType        */ MCUType::MCU_ARDUINO );

//Set the driver type to BTS here
MotorConfig mc{ DriverType::BTS, LF, LB, RF, RB };

/*If you don't want to use dynamic adjustment feature of the PID
gains, then keep the minimum and maximum values of Kp and Kd equal.*/
PIDConfig pc{
  Kp_min,      // The optimal Kp value for straight lines
  Kp_max,      // The optimal Kp value for curved lines
  Ki,          // Generally Ki does not need to vary dynamically so it has one constant value
  Kd_min,      // The optimal Kd value for straight lines
  Kd_max,      // The optimal Kd value for curved lines
  base_speed,  // The speed at which the robot will run on straight lines
  min_speed,   // The minimum speed while turning
  max_speed,   // The maximum speed while turning
  reference    // The position value when the sensor is on the middle of the line.
};

REFEasyLFR lfr(sc, mc, pc);

void setup() {
  // put your setup code here, to run once:
  lfr.begin();
  Serial.begin(9600);
  //For the first time use on a track use the calibrate function to calibrate
  lfr.calibrate(5000); // Enter the number of iterations for which the bot will calibrate. 
                       // For Arduinos 5000 is enough, but for PICO it should be 10000 as the clk speed is high.

  /* For further use on the same track you can skip the calibration by 
  reading the calibrated values from the EEPROM. For that case uncomment this line and comment
  the lfr.calibrate()*/

  // lfr.readCalibration(10); //The default starting address is 10
}

void loop() {
  //To run the robot using only the built-in PID control system use this lfr.runOnce() function
  lfr.runOnce();

  /* You can also do this by individually accessing the readLine() and followLine() function.
  This gives you more flexibility if you want to use the position value for implementing other
  logics. To do this just uncomment the following portion and comment the lfr.runOnce()*/

  // uint16_t position = lfr.readLine();
  // lfr.followLine(position);

  /* You can also access the onLine variable to monitor the number of sensors on the line
  and imply other logics. Such as, if the onLine is equal to the number of sensors, the robot 
  might have reached an end point consisting a black box*/

  while (lfr.onLine() == numSensor) {
    lfr.drive(0, 0);  //If the robot needs to stop at the end point. Apply logics according to your requirement.
  }
}
