/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital
  pin 13, when pressing a pushbutton attached to pin 2.


  The circuit:
   LED attached from pin 13 to ground
   pushbutton attached to pin 2 from +5V
   10K resistor attached to pin 2 from ground

   Note: on most Arduinos there is already an LED on the board
  attached to pin 13.


  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Button
*/

// constants won't change. They're used here to
// set pin numbers:
const int BB_DIpin1 = 24;
const int BB_DIpin2 = 25;
const int A_DIpin = 38;
const int AutoMan_DIpin = 51;

const int stand_LED_PIN = 13;
const int ALED_1_PIN = 44;
const int ALED_2_PIN = 45;
const int ALED_3_PIN = 46;

// variables will change:
int BB_DIState1 = 0;         // variable for reading the pushbutton status
int BB_DIState2 = 0;
int A_DIState = 0;
int AutoMan_DIState = 0;

void setup() {
  pinMode(stand_LED_PIN, OUTPUT);
  pinMode(ALED_1_PIN, OUTPUT);
  pinMode(ALED_2_PIN, OUTPUT);
  pinMode(ALED_3_PIN, OUTPUT);

  pinMode(BB_DIpin1, INPUT);
  pinMode(BB_DIpin2, INPUT);
  pinMode(A_DIpin, INPUT);
  pinMode(AutoMan_DIpin, INPUT);
}

void loop() {
  // read the state
  BB_DIState1 = digitalRead(BB_DIpin1);
  BB_DIState2 = digitalRead(BB_DIpin2);
  A_DIState = digitalRead(A_DIpin);
  AutoMan_DIState = digitalRead(AutoMan_DIpin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (BB_DIState1 == HIGH) {
    digitalWrite(stand_LED_PIN, HIGH);
  } else {
    digitalWrite(stand_LED_PIN, LOW);
  }
  if (BB_DIState2 == HIGH) {
    digitalWrite(ALED_1_PIN, HIGH);
  } else {
    digitalWrite(ALED_1_PIN, LOW);
  }
  if (A_DIState == HIGH) {
    digitalWrite(ALED_2_PIN, HIGH);
  } else {
    digitalWrite(ALED_2_PIN, LOW);
  }
  if (AutoMan_DIState == HIGH) {
    digitalWrite(ALED_3_PIN, HIGH);
  } else {
    digitalWrite(ALED_3_PIN, LOW);
  }
}
