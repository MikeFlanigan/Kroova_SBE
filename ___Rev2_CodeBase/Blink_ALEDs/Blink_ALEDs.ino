/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
*/

const int stand_LED_PIN = 13;
const int ALED_1_PIN = 44;
const int ALED_2_PIN = 45;
const int ALED_3_PIN = 46;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(stand_LED_PIN, OUTPUT);
  pinMode(ALED_1_PIN, OUTPUT);
  pinMode(ALED_2_PIN, OUTPUT);
  pinMode(ALED_3_PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(stand_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(3000);              // wait for a second
  digitalWrite(stand_LED_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(3000);              // wait for a second
  digitalWrite(ALED_1_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(3000);              // wait for a second
  digitalWrite(ALED_1_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(3000);              // wait for a second
  digitalWrite(ALED_2_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(3000);              // wait for a second
  digitalWrite(ALED_2_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(3000);              // wait for a second
  digitalWrite(ALED_3_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(3000);              // wait for a second
  digitalWrite(ALED_3_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(3000);              // wait for a second
}
