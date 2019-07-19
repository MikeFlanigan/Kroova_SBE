

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define MOTHMIN 300 // temp
#define MOTHMAX 310 // temp

float flap_percent = 0.0; // command range between 0.0 and 1.0
uint16_t flap_cmd_pulse = 300; // set this to mid range

// servo #
uint8_t servonum = 8;

void setup() {
  Serial.begin(9600);
  Serial.println("Servo test!");

  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);

  //  for (flap_percent = 0; flap_percent <= 1.0; flap_percent += 0.001) {
  //    // check resolution here
  //    flap_cmd_pulse = map(flap_percent, 0.0, 1.0, MOTHMIN, MOTHMAX)
  //  }
  //  if (SERVOMIN < flap_cmd_pulse < SERVOMAX) {
  //    pwm.setPWM(servonum, 0, flap_cmd_pulse);
  //  }
  //  delay(500);
  //  for (flap_percent = 1.0; flap_percent >= 1.0; flap_percent -= 0.001) {
  //    // check resolution here
  //    flap_cmd_pulse = map(flap_percent, 0.0, 1.0, MOTHMIN, MOTHMAX)
  //  }
  //  if (SERVOMIN < flap_cmd_pulse < SERVOMAX) {
  //    pwm.setPWM(servonum, 0, flap_cmd_pulse);
  //  }
  //  delay(500);
}
