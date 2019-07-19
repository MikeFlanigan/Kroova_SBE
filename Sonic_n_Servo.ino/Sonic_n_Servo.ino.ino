/*
  Multiple Serial test

  Receives from the main serial port, sends to the others.
  Receives from serial port 1, sends to the main serial (Serial 0).

  This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc.

  The circuit:
  - any serial device attached to Serial port 1
  - Serial Monitor open on Serial port 0

  created 30 Dec 2008
  modified 20 May 2012
  by Tom Igoe & Jed Roach
  modified 27 Nov 2015
  by Arturo Guadalupi

  This example code is in the public domain.
*/

#include <Servo.h>
Servo myservo;  // create servo object to control a servo

int out_angle = 65;    // variable to store the servo position
int max_angle = 135;
int min_angle = 70; 

// note Max3232 chip runs on 3.3V power from the mega
int count = 0;

int target_rh = 647; // mm
int error = 0; // dist from target RH
int sum_error = 0; 
float p_gain = -0.20;
float i_gain = -0.002;

int dist = 0; // mm

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial1.begin(9600);

  myservo.attach(9);
  myservo.write(out_angle);
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial1.available()) {
    int inByte = Serial1.parseInt();
    dist = inByte*25.4*0.003384;
//    char inByte = (char)Serial1.read();
//    int inByte = Serial1.read();
//    Serial.println(dist);
    count++;
//    Serial1.flush();
  }
  else{
    Serial.print(count);
    Serial.println("no data");
  }

  error = dist - target_rh;
  
  out_angle = error*p_gain + sum_error*i_gain + 90;
  if (out_angle > max_angle) {
    out_angle = max_angle;
    Serial.print("max");
  }
  else if (out_angle < min_angle) {
    out_angle = min_angle;
    Serial.print("min");
  }
  else{
    sum_error += error; // only integrate the error if the system is not at max corrective action already to prevent wind up
  }
  myservo.write(out_angle);
  Serial.print(error);
  Serial.print(" ");
  Serial.println(out_angle);
  

  // read from port 0, send to port 1:
//  if (Serial.available()) {
//    int inByte = Serial.read();
//    Serial1.write(inByte);
//  }
}
