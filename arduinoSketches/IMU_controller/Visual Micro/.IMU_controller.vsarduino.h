//Board = Arduino Uno
#define __AVR_ATmega328P__
#define ARDUINO 105
#define F_CPU 16000000L
#define __AVR__
extern "C" void __cxa_pure_virtual() {;}

//
//
void fillIdentityMatrix();

#include "C:\Program Files (x86)\Arduino\hardware\arduino\variants\standard\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "W:\quadcopter\arduinoSketches\IMU_controller\IMU_controller.ino"
#include "W:\quadcopter\arduinoSketches\IMU_controller\ADXL345.cpp"
#include "W:\quadcopter\arduinoSketches\IMU_controller\ADXL345.h"
#include "W:\quadcopter\arduinoSketches\IMU_controller\HMC5883L.cpp"
#include "W:\quadcopter\arduinoSketches\IMU_controller\HMC5883L.h"
#include "W:\quadcopter\arduinoSketches\IMU_controller\L3G4200D.cpp"
#include "W:\quadcopter\arduinoSketches\IMU_controller\L3G4200D.h"
