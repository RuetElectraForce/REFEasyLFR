Usage Notes
===========

This document shows how to use the various parts of the **REFEasyLFR** library:  
- **SensorReader** (with MUX or direct-read)  
- **MotorDriver** (BTS or L298)  
- **LineController** (dynamic PID)  
- **REFEasyLFR** facade (all-in-one interface)  

---

## 1. SensorReader

### 1.1. Calibration

This library allows you to use the SensorReader::calibrate() method to easily calibrate your sensors for the particular environment it will encounter. Calibrating your sensors can lead to substantially more reliable sensor readings, which in turn can help simplify your code. As such, we recommend you build a calibration phase into your application’s initialization routine. This can be done very easily by calling the `calibrate` method inside the `setup()` function. During this calibration phase, you will need to expose each of your reflectance sensors to the lightest and darkest readings they will encounter. For example, if you have made a line follower, you will want to slide it across the line during the calibration phase so the each sensor can get a reading of how dark the line is and how light the ground is. 
This library is also provides the saving feature of the calibrated values. You just have to use the `saveCalibration` method with an initial saving address. By default the `calibrate` method saves the values from address `10`. If you are running your robot on the same environment there is no need to recalibrated. Simply use the method `readCalibration` with the same initial address that was used to save (For default, use `10` as the initial addess).
A sample calibration and savng routine would be:

```cpp
#include <SensorReader.h>
/* This library comes up with the feature to use your reflectance
sensor with or withour multiplexer(MUX). If you are not using the MUX
then uncomment the `useDirect` type configuration and remove/comment 
the `useMux` type configuration*/
SensorConfig sc = SensorConfig::useMux(
  /* Always check the sensor output sequence after setting all up.
     If the sensor output sequence is out of order then reverse the
     select line pin sequence here(i.e.: S0, S1, S2, S3)*/
  /* S3,S2,S1,S0 */ 2,3,4,5,
  /* SIG pin      */ A0,
  /* numSensors   */ 14,

  /*Set the invert true if the robot will possibly encounter white line on black surface*/
  /* invert?      */ false,
  /* flagMinTh    */ 50,
  /* flagMaxTh    */ 200,

  /*Default is MCU_ARDUINO. It is optional to initialize unless you are using Pi Pico.
   Use "MCUType::PICO" instead for that case.*/
  /* mcuType      */ MCUType::MCU_ARDUINO 
);

// // _______Direct reading Upto eight analog pins A0–A7 (Upto 16 analog pins if ATMega2560 is used):_______
// /* If you are using Pi Pico or other MCU that does not have enough analog pins
// you cannot use the direct method.*/
// const uint8_t pins[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
// SensorConfig sc = SensorConfig::useDirect(
// /* All input pins */pins,
// /* numSensors     */ 8,
// /* flagMinTh      */false,
// /* flagMinTh      */ 50,
// /* flagMaxTh      */ 200,
// /* mcuType        */ MCUType::MCU_ARDUINO );

//Make an object of the SensorReader class
SensorReader sr(sc);

void setup() {
  Serial.begin(9600);
  sr.begin();
  sr.calibrate();   //This function automatically saves the calibrated values from address 10
}

void loop() {
  // …
}
```
For second time use in the same environment the code snippet should be:
```cpp
void setup() {
  Serial.begin(9600);
  sr.begin();
  sr.readCalibration(10);   //Used the default address 10 for reading the saved values
}

void loop() {
  // …
}
```

### 1.2. Reading the Sensors
This library made sensor reading very easy on both black lines and white lines by using a simple command; SensorReader::readLine() which returns a relative postion value on the line. 

1. If your sensor will be encountering white line on black surface then set the `invert` to `true` otherwise `false`.

2. You can request the ccalibrated sensor values any time of testing using the SensorReader::printCalibration() after calibrating or reading the calibrated values from EEPROM in the setup() function.

3. You can use the SensorReader::printPosition() method to observe each sensor values and the relative position of the sensor on the line. The range of the relative positon value depends on the number of IRs that the sensor array consists of. If you are using N sensors, a returned value of 0 means it thinks the line is on or to the outside of sensor 0, and a returned value of 1000 &times; (N&minus;1) means it thinks the line is on or to the outside of sensor N&minus;1. As you slide your sensors across the line, the line position will change monotonically from 0 to 1000 &times; (N&minus;1), or vice versa. This line-position value can be used for closed-loop PID control.

A sample routine to obtain the sensor values and observe the position value on the line would be:

```cpp
//after initalizing the parameters like above you can start using the methods using the object 'sr'
void loop() {
  //This method prints each sensor values, the relative position and, the number of sensors that are on the line.
  sr.printPosition(); 
}
```

## 2. MotorDriver
This library allows the user to use two types of motor drivers, i.e.: BTS7960B (Requires 4 PWM pins) and L298N or similar (Requires 4 digital pins and 2 PWM pins). The library uses only one basic method MotorDirver::drive(int leftSpeed, rightSpeed) which receives two integer values representing the 8-bit PWM value (0-255).

### 2.1. BTS7960B Setup and Usage
For the setup portion carefully declare the forward and backward PWM pins. If positive PWM values are provided inside the `drive(255, 255)` but the motor spins in the reverse direction from the desired direction, simply swap that motor's pin numbers with each other.

A sample routine to run the BTS7960B motor driver would be:

```cpp
#include<MotorDriver.h>

//___________Motor Pins____________
//Make sure to check the PWM capability of the pins of your corresponding Microcontroller
#define LF  6  // Left forward pin 
#define LB  9  // Left backward pin
#define RF  10 // Right forward pin
#define RB  11 // Right backward pin

//Set the driver type to BTS here
MotorConfig mc {DriverType::BTS, LF, LB, RF, RB};

//Make an object of the MotorDriver class
MotorDriver md(mc);

void setup() {
  md.begin();
}

void loop() {
  //spin the motor in forward direction for 2 seconds
  md.drive(255, 255);
  delay(2000);

  //spin the motor in reverse direction for 2 seconds
  md.drive(-255, -255);
  delay(2000);

  //Test speed increment
  for(int speed = 0; speed += 5; speed <= 255)
  {
    drive(speed, speed);
    delay(100);
  }

  //Test speed increment
  for(int speed = 0; speed -= 5; speed >= -255)
  {
    drive(speed, speed);
    delay(100);
  }
}
```

### 2.2. L298N or Similar Motor Driver Setup
To setup these type of motor drivers, you need to declare 4 direction pins (any digital pin), and 2 PWM pins. If positive PWM values are provided inside the `drive(255, 255)` but the motor spins in the reverse direction from the desired direction, simply swap that motor's direction pin numbers with each other. 
The initialization routine for these motors would be:

```cpp
#include<MotorDriver.h>

//___________Motor Pins____________
//Make sure to check the PWM capability of the pins of your corresponding Microcontroller
#define LF  3  // Left forward pin 
#define LB  4  // Left backward pin
#define RF  5 // Right forward pin
#define RB  6 // Right backward pin

#define LPWM  9 // Left PWM pin
#define RPWM  10 // Right PWM pin

//Set the driver type to L298 here
MotorConfig mc {DriverType::L298, LF, LB, RF, RB, LPWM, RPWM};

//Make an object of the MotorDriver class
MotorDriver md(mc);

void setup() {
  md.begin();
}

void loop() {
  //..Rest of the usage is same as above.
}
```

## 3. LineController
This library has made the application of PID control system in line-following robots very easy. You need to initialize the PID parameters and use the LineController::followLine(int position) method to make the robot to follow the line. The performance of the robot on the line will depend on how well the PID gains are tuned. 
An additional feature has been to our PID controller in this library. Unlike the traditional PID system where the gains (Kp, Kd, Ki) are static, in our system the gains(Kp, Kd) vary between a pre-defined range. This is done because, a constant set of gains are never optimal for both straigh lines and curved lines. The minimum value of the constants provide better performance on the straigh lines whereas the maximum value of the constants performs well on the tight curves and sharp turns. 
To perfectly utilize this feature first, you have to tune your robot to run smoothly on the straight line and record the values as minimum. Then the robot needs to be tuned for better performance on the tight curves and record those values as maximum. Finally put these values at the initialization part.

The initialization portion would look like:

```cpp
#include <LineController.h>
/*  During the individual tunning for straight and curved lines, 
keep the minimum and maximum values of Kp & Kd equal. After finalizing
the values assign the ranges. Here some dummy values are provided. 
Your values will obviously differ from these but will be comparable,
i.e.: The Kp_min/Kp_max will not be 10, 20, 100 or vice versa.

The minimum and maximum speeds are the speeds that the motor will vary 
from it's base speed during the actual run. 

The reference value depends on the number of IRs on your array. 
Generally it is, ((num_sensor - 1) * 1000) / 2.
*/
float Kp_min = 0.02, Kp_max = 0.08, Kd_min = 4.0, Kd_max = 8.0, Ki = 0.001;
int base_speed = 100, min_speed = -40, max_speed = 180;
uint16_t reference = 6500;

PIDConfig pc {
  Kp_min,     // The optimal Kp value for straight lines
  Kp_max,     // The optimal Kp value for curved lines
  Ki,         // Generally Ki does not need to vary dynamically
  Kd_min,     // The optimal Kd value for straight lines
  Kd_max,     // The optimal Kd value for curved lines
  base_speed, // The speed at which the robot will run on straight lines
  min_speed,  // The minimum speed while turning
  max_speed,  // The maximum speed while turning
  reference   // The position value when the sensor is on the middle of the line.
}

//Make the object of LineController class
LineController lc(pc);

void setup() {
  //Initialize the LineController
  lc.begin();
}

void loop {
  //...
}
```

## 4. REFEasyLFR
This is the overhead library which contains all of the above and while setting up the full working robot you'll use this header to simply access all the features. 

The general outline to use all the functionalities is presented below considering Arduino Nano and L298 as the optional hardware of choice.

```cpp
#include <REFEasyLFR.h>
//_______Mux Variables_______
/* Keep in mind that, for Arduino only A0-A5 pins has digital
functionalities beside analog but A6 & A7 offers only analog input*/
#define S0 A0
#define S1 A1
#define S2 A2
#define S3 A3
#define SIG A7

//_________Motor pins__________
#define RF   12  //right forward
#define RB   11  //right backward
#define RPWM 10  //right PWM
#define LF   7   //left backward
#define LB   8   //left forward
#define LPWM 9   //left PWM

//__________IR reading variables_________
#define numSensor 14

//_________PID variables__________
float Kp_min = 0.02, Kp_max = 0.08, Kd_min = 4.0, Kd_max = 8.0, Ki = 0.001;
int base_speed = 100, min_speed = -40, max_speed = 180;
uint16_t reference = 6500;

//Here sensor reading through MUX is considered
SensorConfig sc = SensorConfig :: useMux(
  S0, S1, S2, S3, SIG, 14, false, 100, 250);

MotorConfig mc {DriverType::L298, LF, LB, RF, RB, LPWM, RPWM};
PIDConfig pc {Kp_min, Kp_max, Ki, Kd_min, Kd_max, base_speed, min_speed, max_speed, reference};

/*Make and object of the REFEasyLFR class while passing all the configuration structures to the REFEasyLFR class. This 
"lfr" object can be used to access all the methods available
in SensorReader, MotorDriver and, LineController*/
REFEasyLFR lfr(sc, mc, pc);

void setup() {
  // put your setup code here, to run once:
  lfr.begin();
  Serial.begin(9600);
  //For the first time use on a track use the calibrate function to calibrate
  lfr.calibrate();

  /* For further use on the same track you can skip the calibration by 
  reading the calibrated values from the EEPROM. For that case uncomment this line and comment
  the lfr.calibrate()*/

  // lfr.readCalibration(10);
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
    lfr.drive(0,0); //If the robot needs to stop at the end point. Apply logics according to your requirement.
  }
}
```
## 5. Circuit Diagram
The overall circuit diagram with an Arduino Nano would look like this ([Click here to view detailed circuit](https://app.cirkitdesigner.com/project/8df65339-e6b2-4c45-b0c9-fccbe6c56bb0)). Notice that, in this circuit an OLED display and pushbuttons are added. You can use those components to make a menu system in which you can also use all the functionalities of the `REFEasyLFR` library. 
![REFEasyLFR ckt](https://github.com/user-attachments/assets/602c9c49-00e5-4537-9865-95e1aad5f722)

If you intend to change the PID gain values, `base_speed`, `max_speed`, `min_speed` or the `reference` through the menu system, remember to update the variables in the PIDConfig structure and reinitialize REFEasyLFR.
Example: Updating the `baseSpeed`, `KpMin` through any menu function:
```cpp
pc.baseSpeed = 150;
pc.KpMin = 0.05;
REFEasyLFR(sc, mc, pc);
```
N.B.: More details about the menu system coming soon!!


