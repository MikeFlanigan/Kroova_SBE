#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

unsigned long now_millis = millis();

int US_dist = 0;
int incomingByte = 0;
int inByte = 0;
// --------------- GPIO setup ---------------
const int ALED_1_PIN = 44;
const int ALED_2_PIN = 45;
const int ALED_3_PIN = 46;

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
// ----- End of -- GPIO setup ----------------

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

  if (not IMU_cal) {
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
      else Serial.print("sensors calibrated, but not system... rare");
    }
    // temp for debugging
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
  }
  else if (IMU_biased < 2) {
    if (not one_shot) {
      ALED_1 = false; ALED_2 = false; ALED_3 = false;
      one_shot = true;
    }
    if ((not switch1 && now_millis - LED1_millis > 1000) || (switch1 && now_millis - LED1_millis > 500)) {
      LED1_millis = now_millis;
      ALED_1 = not ALED_1;
      ALED_2 = not ALED_2;
      ALED_3 = not ALED_3;
    }
    if (switch1) {
      if (now_millis - IMU_bias_millis < 45000 ) { // 60 second initialization period
        IMU_RH_offset = IMU_RH_offset * (1 - IMU_bias_avg_weight) + IMU_bias_avg_weight * vaccel.z() ; // very slow smoothing
      }
      else { // bias offset collection finished
        IMU_biased = 1;
        if (now_millis - LED1_millis > 150) {
          LED1_millis = now_millis;
          ALED_1 = not ALED_1; ALED_2 = not ALED_2; ALED_3 = not ALED_3;
        }
      }
    }
    else {
      IMU_bias_millis = now_millis; // bias collection switch hasn't been pressed yet
      if (IMU_biased == 1) IMU_biased = 2; // finished
    }
    Serial.print(vaccel.z());
    Serial.print(" ");
    Serial.println(IMU_RH_offset);
  }
  else {
    if ((system == 3 || system == 2) && now_millis - LED1_millis > 1000) { // IMU good indicator
      LED1_millis = now_millis;
      ALED_1 = not ALED_1;
    }
    else if (system < 2 && (now_millis - LED1_millis > 200)) { // IMU error indicator
      LED1_millis = now_millis;
      ALED_1 = not ALED_1;
    }
    if (GPS.fix) {
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
    else {
      if (now_millis - LED3_millis > 200) { // GPS error indicator
        LED3_millis = now_millis;
        ALED_3 = not ALED_3;
      }
      Serial.print("No GPS fix");
    }

    if (Serial3.available()) {
      incomingByte = Serial3.read();
      inByte = Serial3.parseInt();
      US_dist = inByte * 25.4 * 0.003384; // data coming out in mm
      //      Serial3.flush(); // is this important?
    }
    else US_dist = 9999;

    // send to BB
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
    Serial1.print(vaccel.z()-IMU_RH_offset); // sending unbiased value to keep speed high
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
  }


  // -------- write LED outputs --------
  if (ALED_1) digitalWrite(ALED_1_PIN, HIGH);
  else digitalWrite(ALED_1_PIN, LOW);
  if (ALED_2) digitalWrite(ALED_2_PIN, HIGH);
  else digitalWrite(ALED_2_PIN, LOW);
  if (ALED_3) digitalWrite(ALED_3_PIN, HIGH);
  else digitalWrite(ALED_3_PIN, LOW);

//  delay(BNO055_SAMPLERATE_DELAY_MS); // try straight ditching this
}
