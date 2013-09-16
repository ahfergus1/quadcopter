/* ==========================================================

Single Motor Driver v1.0

Controls a single motor via serial.
Open the serial monitor and the values sent via serial will 
change the motor's throttle

============================================================= */

#include <Servo.h>

Servo esc1;
int throttle;
boolean changed = false;
const int throttlePin = 9;

void setup(){
  throttle = 0;
  Serial.begin(9600);
  esc1.attach(throttlePin);
  esc1.writeMicroseconds(0);
  Serial.println(throttle);
}

void loop() {
  if(changed){
      esc1.writeMicroseconds(throttle);
      changed = false;
  }
}

void serialEvent(){
    throttle = Serial.parseInt();
    Serial.println(throttle);
    while(Serial.available()){
      Serial.read();
    }
    changed= true;
}

