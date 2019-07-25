#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <Adafruit_PWMServoDriver.h>
#include <MatrixMath.h>


// --------------- Kalman Filter setup ---------------
bool first_loop_flag = true;

mtx_type I[3][3]; // 3x3 identity
mtx_type F[3][3]; // state transition matrix
mtx_type Ftr[3][3]; // state transition matrix transposed
mtx_type H[2][3]; // measurement matrix
mtx_type Htr[3][2]; // measurement matrix
mtx_type Qkf[3][3]; // process noise matrix
mtx_type R[2][2]; // measurement noise matrix
mtx_type P[3][3]; // covarience matrix
mtx_type P_t1[3][3];
mtx_type P_minus[3][3]; // predicted covariance matrix
mtx_type P_minus_t1[3][3];
mtx_type x[3][1]; // state vector
mtx_type x_minus[3][1]; // predicted state vector
mtx_type x_t21[2][1];
mtx_type x_2t21[2][1];
mtx_type xtemp[3][1];
mtx_type K[3][2]; // kalman gain
mtx_type K_num[3][2]; // kalman gain temp numerator
mtx_type Ktemp23[2][3];
mtx_type Kden[2][2];
mtx_type Kden_t1[2][2];
mtx_type y[2][1]; // measurement vector
// ----- End of -- Kalman Filter setup ----------------

unsigned long now_millis = millis();
unsigned long delay_timer = millis();

int US_dist = 0;
int incomingByte = 0;
int inByte = 0;
// --------------- GPIO setup ---------------
const int ALED_1_PIN = 44;
const int ALED_2_PIN = 45;
const int ALED_3_PIN = 46;
const int BB_DIpin2 = 25;

bool ALED_1;
bool ALED_2;
bool ALED_3;

unsigned long LED1_millis = millis();
unsigned long LED2_millis = millis();
unsigned long LED3_millis = millis();

const int switch1_PIN = 38;
bool switch1 = false;

const int RHA_Pin = A0;
int RHA = 0;
int RH_init = 1200 + 650; // mm !! plus half the throw of the RHA pot
int RH_setP = RH_init;
// ----- End of -- GPIO setup ----------------

// --------- servo and control setup --------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  175 // SMALLER MORE THROW AFT this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  435 // this is the 'maximum' pulse length count (out of 4096)

#define MOTHMIN 175 // temp
#define MOTHMAX 435 // temp

const int servo_middle = 305;
bool servo_enabled = false;

float flap_percent = 0.5; // command range between 0.0 and 1.0
uint16_t flap_cmd_pulse = 300; // set this to mid range
uint8_t servonum = 8;
unsigned long loop_t = millis();
int delta_t = 0;
float dt = 0.0; // decimal seconds
float P_gain = 0.001;
int err = 0; // error in mm
int ctrl_signal = 650; // defaulting
int last_ctrl_signal = 650;
int counterr = 0;
// ------------- end of servo and control setup ---------
// --------------- IMU setup ---------------
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055();

uint8_t system, gyro, accel, mag = 0;
bool IMU_cal = false;
int IMU_biased = 0;

float IMU_bias_avg_weight = 0.004; // larger converges faster but is more sensitive to noise
unsigned long IMU_bias_millis = millis();
float IMU_RH_offset = 0.0;

bool one_shot = false;

float Heave_acc = 0.0;
// ----- End of -- IMU setup ----------------

// --------------- GPS setup ---------------
Adafruit_GPS GPS(&Serial2);
HardwareSerial mySerial = Serial2;
#define GPSECHO  false
boolean usingInterrupt = true;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
float Lat = 0.0;
float Lon = 0.0;
float Speed = 0.0;
// ----- End of -- GPS setup ----------------

void setup(void)
{
  pinMode(ALED_1_PIN, OUTPUT);
  pinMode(ALED_2_PIN, OUTPUT);
  pinMode(ALED_3_PIN, OUTPUT);
  pinMode(switch1_PIN, INPUT_PULLUP);
  pinMode(BB_DIpin2, INPUT);

  Serial.begin(115200); // console debugging
  Serial1.begin(115200); // BeagleBone comms !!!!!!!!!!!!! should this be at 115200?
  GPS.begin(9600); // GPS on serial 2
  Serial3.begin(9600); // ultrasonic

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // --------------- Kalman Filter init ---------------
  for (int i = 0; i < 3; i++) { // rows
    for (int j = 0; j < 3; j++) { // cols
      // ------- init I
      if (i == j) I[i][j] = 1.0f;
      else I[i][j] = 0.0f;
      // ------- init H
      if (i < 2) {
        if (i == 0 && j == 0) H[i][j] = 1.0f;
        else if (i == 1 && j == 2) H[i][j] = 1.0f;
        else H[i][j] = 0.0f;
      }
      // ------- init Qkf and R
      if (i == j) {
        if (i == 0) {
          Qkf[i][j] = 0.1f;
          R[i][j] = 0.1f;
        }
        else if (i == 1) {
          Qkf[i][j] = 0.01f;
          R[i][j] = 0.01f;
        }
        else if (i == 2) Qkf[i][j] = 0.01f;
      }
      else {
        if (i < 2) R[i][j] = 0.0f;
        Qkf[i][j] = 0.0f;
      }
    }
  }
  // ----- End of -- Kalman Filter init ----------------

  pwm.begin();
  pwm.setPWMFreq(60);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  useInterrupt(true);

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void loop(void)
{
  now_millis = millis();

  switch1 = not digitalRead(switch1_PIN);
  RHA = analogRead(RHA_Pin);
  RH_setP = RH_init - map(RHA, 140, 600, 0, 1000);
  servo_enabled = digitalRead(BB_DIpin2);

  if (GPS.newNMEAreceived()) {
    Lat = GPS.latitudeDegrees;
    Lon = GPS.longitudeDegrees;
    Speed = GPS.speed; // knots
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> vaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  bno.getCalibration(&system, &gyro, &accel, &mag);

  //  IMU_cal = true; // debugFLAG DEBUGGINGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG
  //  IMU_biased = 3; // debugFLAG DEBUGGINGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG
  if (not IMU_cal) { // initial IMU calibration
    if (gyro < 3 || accel < 3 || mag < 3) {
      if (now_millis - LED1_millis > 1000 / (4 - accel)) {
        LED1_millis = now_millis;
        ALED_1 = not ALED_1;
      }
      if (accel == 3) ALED_1 = false;
      if (now_millis - LED2_millis > 1000 / (4 - gyro)) {
        LED2_millis = now_millis;
        ALED_2 = not ALED_2;
      }
      if (gyro == 3) ALED_2 = false;
      if (now_millis - LED3_millis > 1000 / (4 - mag)) {
        LED3_millis = now_millis;
        ALED_3 = not ALED_3;
      }
      if (mag == 3) ALED_3 = false;
    }
    else { // accel gyro and mag ready
      if (system == 3 || system == 2) IMU_cal = true;
      //      else Serial.print("sensors calibrated, but not system... rare");
      ALED_1 = false;
      ALED_2 = false;
      ALED_3 = false;
    }
    // temp for debugging
    //    Serial.print("CALIBRATION: Sys=");
    //    Serial.print(system, DEC);
    //    Serial.print(" Gyro=");
    //    Serial.print(gyro, DEC);
    //    Serial.print(" Accel=");
    //    Serial.print(accel, DEC);
    //    Serial.print(" Mag=");
    //    Serial.println(mag, DEC);

    delay_timer = millis();
  }
  else if (IMU_biased < 2) { // IMU offset gathering
    if (now_millis - delay_timer < 30000) {
      if (now_millis - LED1_millis > 2000) {
        LED1_millis = now_millis;
        ALED_1 = not ALED_1;
        ALED_2 = not ALED_2;
      }
      IMU_bias_millis = millis();
    }
    else {
      if (now_millis - LED1_millis > 500) {
        LED1_millis = now_millis;
        ALED_1 = not ALED_1;
        ALED_2 = not ALED_2;
      }
      //      Serial.print(" Collecting offset data... ");
      if (now_millis - IMU_bias_millis < 45000 ) { // 60 second initialization period
        IMU_RH_offset = IMU_RH_offset * (1 - IMU_bias_avg_weight) + IMU_bias_avg_weight * vaccel.z() ; // very slow smoothing
      }
      else { // bias offset collection finished
        IMU_biased = 2;
        LED1_millis = now_millis;
        ALED_1 = false;
        ALED_2 = false;
      }

    }
    //    Serial.print(vaccel.z());
    //    Serial.print(" ");
    //    Serial.print(now_millis - IMU_bias_millis);
    //    Serial.print(" ");
    //    Serial.println(IMU_RH_offset);
  }

  else { // system is done with calibration
    if ((system == 3 || system == 2) && now_millis - LED1_millis > 1000) { // IMU good indicator
      LED1_millis = now_millis;
      ALED_1 = not ALED_1;
    }
    else if (system < 2 && (now_millis - LED1_millis > 200)) { // IMU error indicator
      LED1_millis = now_millis;
      ALED_1 = not ALED_1;
      //      Serial.print(" IMU out of cal "); // DEBUGGGGGGGINGGGGGGGGGGGGGGGGGGGGGGGGG
    }
  }

  if (GPS.fix) {
    if (IMU_cal) {
      if (now_millis - LED3_millis > 1000) { // GPS good indicator
        LED3_millis = now_millis;
        ALED_3 = not ALED_3;
      }

      //      Serial.print(GPS.hour);
      //      Serial.print(" ");
      //      Serial.print(GPS.minute);
      //      Serial.print(" ");
      //      Serial.print(GPS.seconds);
      //      Serial.print(" ");
      //      Serial.print(GPS.speed);
      //      Serial.print(" ");
      //      Serial.print(GPS.latitudeDegrees);
      //      Serial.print(" ");
      //      Serial.print(GPS.longitudeDegrees);
    }

  }
  else {
    if (IMU_cal) {
      if (now_millis - LED3_millis > 200) { // GPS error indicator
        LED3_millis = now_millis;
        ALED_3 = not ALED_3;
      }
    }
    //    Serial.print(" No GPS fix ");
  }



  if (Serial3.available()) {
    incomingByte = Serial3.read();
    inByte = Serial3.parseInt();
    US_dist = inByte * 25.4 * 0.003384; // data coming out in mm
    //      Serial3.flush(); // is this important?
  }
  else US_dist = 9999;

  // --------------- control ----
  delta_t = millis() - loop_t;
  dt = (float) delta_t / 1000;
  loop_t = millis();

  last_ctrl_signal = ctrl_signal;
  if (US_dist < 10) {
    ctrl_signal = last_ctrl_signal;
  }
  else {
    ctrl_signal = US_dist;
  }

  Heave_acc = vaccel.z() - IMU_RH_offset;

  // ------------ Kalman Filter calcs ------------
  if (first_loop_flag) { // collect data and initialize KF variables
    first_loop_flag = false;
    x[0][0] = ctrl_signal;
    x[1][0] = 0.0f;
    x[2][0] = Heave_acc;
  }
  else { // variables have been initialized on the first loop
    // recompute F since it's time dependent
    for (int i = 0; i < 3; i++) { // rows
      for (int j = 0; j < 3; j++) { // cols
        if (i == j) F[i][j] = 1.0f;
        else if (i > j) F[i][j] = 0.0f;
        else if (i == 2 && j == 2) F[i][j] = 0.5 * dt * dt;
        else if (j > 0 && i < j) F[i][j] = dt;
      }
    }

    // Perform prediction step
    Matrix.Multiply((mtx_type*)F, (mtx_type*)x, 3, 3, 1, (mtx_type*)x_minus);

    Matrix.Transpose((mtx_type*)F, 3, 3, (mtx_type*)Ftr);
    Matrix.Multiply((mtx_type*)F, (mtx_type*)P, 3, 3, 3, (mtx_type*)P_minus);
    Matrix.Multiply((mtx_type*)P_minus, (mtx_type*)Ftr, 3, 3, 3, (mtx_type*)P_minus_t1);
    Matrix.Add((mtx_type*)P_minus_t1, (mtx_type*)Qkf, 3, 3, (mtx_type*)P_minus);

    // Compute Kalman gain
    Matrix.Transpose((mtx_type*)H, 2, 3, (mtx_type*)Htr);
    Matrix.Multiply((mtx_type*)P_minus, (mtx_type*)Htr, 3, 3, 2, (mtx_type*)K_num);
    Matrix.Multiply((mtx_type*)H, (mtx_type*)P_minus, 2, 3, 3, (mtx_type*)Ktemp23);
    Matrix.Multiply((mtx_type*)Ktemp23, (mtx_type*)Htr, 2, 3, 2, (mtx_type*)Kden_t1);
    Matrix.Add((mtx_type*)Kden_t1, (mtx_type*)R, 2, 2, (mtx_type*)Kden);
    Matrix.Invert((mtx_type*)Kden, 2);
    Matrix.Multiply((mtx_type*)K_num, (mtx_type*)Kden, 3, 2, 2, (mtx_type*)K);

    // gather measurement vector
    y[0][0] = ctrl_signal;
    y[1][0] = Heave_acc;

    // Prediction correction step
    Matrix.Multiply((mtx_type*)H, (mtx_type*)x_minus, 2, 3, 1, (mtx_type*)x_t21);
    Matrix.Subtract((mtx_type*)y, (mtx_type*)x_t21, 2, 1, (mtx_type*)x_2t21);
    Matrix.Multiply((mtx_type*)K, (mtx_type*)x_2t21, 3, 2, 1, (mtx_type*)xtemp);
    Matrix.Add((mtx_type*)x_minus, (mtx_type*)xtemp, 3, 1, (mtx_type*)x);

    Matrix.Multiply((mtx_type*)K, (mtx_type*)H, 3, 2, 3, (mtx_type*)P);
    Matrix.Subtract((mtx_type*)I, (mtx_type*)P, 3, 3, (mtx_type*)P_t1);
    Matrix.Multiply((mtx_type*)P_t1, (mtx_type*)P_minus, 3, 3, 3, (mtx_type*)P);
  }
  if (x[0][0] < 10) x[0][0] = 10; // min threshold on RH
  else if (x[0][0] > 3000) x[0][0] = 3000; // max threshold on RH

  ctrl_signal = x[0][0];
  // -- end of -- Kalman Filter calcs ------------

  err = RH_setP - ctrl_signal;
  flap_percent = 1 * P_gain * err + 0.2; // add bias to account for neutral lift offset

  if (servo_enabled) {
    flap_cmd_pulse = flap_percent * (MOTHMAX - MOTHMIN) + MOTHMIN;
    flap_cmd_pulse = map(flap_cmd_pulse, MOTHMIN, MOTHMAX, MOTHMAX, MOTHMIN);
  }
  else {
    if (abs(map(RHA, 140, 600, MOTHMIN, MOTHMAX) - flap_cmd_pulse) > 15) { // prevents jittering since the servo should be disabled
          flap_cmd_pulse = map(RHA, 140, 600, MOTHMIN, MOTHMAX);
    }
//    flap_cmd_pulse = servo_middle;
    x[1][0] = 0.0; // clamp the IMU velocity term to zero when the servo isn't enabled
  }
  if (SERVOMIN < flap_cmd_pulse && flap_cmd_pulse < SERVOMAX) {
    // debugging
    //    Serial.print(" ctrl sig: ");
//    Serial.println(ctrl_signal);
    //    Serial.print(" dt: ");
    //    Serial.println(delta_t);
    //    Serial.print(" US dist: ");
    //    Serial.print(US_dist);
    //    Serial.print(" ctrl sig: ");
    //    Serial.print(ctrl_signal);
    //    Serial.print(" flap percent: ");
    //    Serial.print(flap_percent);
    //    Serial.print(" flap command pulse: ");
    //    Serial.print(flap_cmd_pulse);
    // end of debugging
    pwm.setPWM(servonum, 0, flap_cmd_pulse);
  }
  // end control --------------

  // send to BB
  Serial1.print(ctrl_signal);
  Serial1.print(",");
  Serial1.print(flap_percent);
  Serial1.print(",");
  //    Serial1.print(GPS.hour);
  //    Serial1.print(",");
  //    Serial1.print(GPS.minute);
  //    Serial1.print(",");
  //    Serial1.print(GPS.seconds);
  //    Serial1.print(",");
  Serial1.print(GPS.speed);
  Serial1.print(",");
  //    Serial1.print(GPS.latitudeDegrees);
  //    Serial1.print(",");
  //    Serial1.print(GPS.longitudeDegrees);
  //    Serial1.print(",");
  Serial1.print(US_dist);
  Serial1.print(",");
  //    Serial1.print(RHA);
  //    Serial1.print(",");
  //    Serial1.print(IMU_RH_offset);
  //    Serial1.print(",");
  Serial1.print(vaccel.z() - IMU_RH_offset); // sending unbiased value to keep speed high
  Serial1.print(",");
  //    Serial1.print(vaccel.z()); // !!!!!!!!!!!!!!!!!!!!!!!
  //    Serial1.print(",");
  Serial1.print(euler.y());
  Serial1.print(",");
  Serial1.println(euler.z());

  //    Serial.print(" US_RH:");
  //    Serial.print(US_dist);
  //    Serial.print(" RHA:");
  //    Serial.print(RHA);
  //    Serial.print(" heave_acc_bias:");
  //    Serial.print(IMU_RH_offset);
  //    Serial.print(" heave_acc:");
  //    Serial.print(vaccel.z());
  //    Serial.print(" heel:");
  //    Serial.print(euler.y());
  //    Serial.print(" pitch:");
  //    Serial.println(euler.z());

  // -------- write LED outputs --------
  if (ALED_1) digitalWrite(ALED_1_PIN, HIGH);
  else digitalWrite(ALED_1_PIN, LOW);
  if (ALED_2) digitalWrite(ALED_2_PIN, HIGH);
  else digitalWrite(ALED_2_PIN, LOW);
  if (ALED_3) digitalWrite(ALED_3_PIN, HIGH);
  else digitalWrite(ALED_3_PIN, LOW);

  //  delay(BNO055_SAMPLERATE_DELAY_MS); // try straight ditching this
}
