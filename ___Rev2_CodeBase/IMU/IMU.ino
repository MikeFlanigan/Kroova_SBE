#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

const int ALED_1_PIN = 44;

unsigned long last_millis = millis();
unsigned long now_millis = millis();
unsigned long delta_t = 0;

float filt_heave_acc = 0.0;
float weight = 0.95;

float heave_vel = 0.0;

float heave = 0.0;

float heave_acc = 0.0;

float heave_acc_offset = 0.0;
float heave_acc_roll_avg_weight = 0.005; // larger converges faster but is more sensitive to noise
unsigned long accel_init_timer = millis();
bool accel_init = false;

bool oneshot = false;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055();

uint8_t system, gyro, accel, mag = 0;
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(ALED_1_PIN, OUTPUT);

  Serial1.begin(9600);
  Serial2.begin(9600);

  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

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

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  now_millis = millis();

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> vaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> vgrav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  //  if ((system == 3 && gyro == 3 && accel == 3 && mag == 3) || (system == 3 && accel == 3)) {
  if ((system == 3 && gyro == 3 && accel == 3 && mag == 3) || (system == 3)) {
    if (oneshot == false) {
      oneshot = true;
      for (int count = 0; count < 6; count++) {
        digitalWrite(ALED_1_PIN, HIGH);
        delay(1000);
        digitalWrite(ALED_1_PIN, LOW);
        delay(1000);
      }
    }
    // will probably need to be made longer or different or based on button press
    digitalWrite(ALED_1_PIN, HIGH);


    /* Display the floating point data */
    //    Serial.print("X: ");
    //    Serial.print(euler.x());
    //    Serial.print(" Y: ");
    //    Serial.print(euler.y());
    //    Serial.print(" Z: ");
    //    Serial.print(euler.z());
    //    Serial.print("\t\t");
    //
    //    Serial.print("X: ");
    //    Serial.print(vaccel.x());
    //    Serial.print(" Y: ");
    //    Serial.print(vaccel.y());
    //    Serial.print(" Z: ");
    //    Serial.print(vaccel.z());
    //    Serial.println("\t\t");

    //    Serial.println(vaccel.x());
    //    Serial1.println(vaccel.y());


    delta_t = now_millis - last_millis;
    last_millis = now_millis;

    weight = 0.2;
    filt_heave_acc = filt_heave_acc * (1 - weight) + vaccel.z() * (weight);

    if (now_millis - accel_init_timer < 61000 && accel_init == false ) { // 60 second initialization period
      heave_acc_offset = heave_acc_offset * (1 - heave_acc_roll_avg_weight) + heave_acc_roll_avg_weight * filt_heave_acc ; // very slow smoothing
      Serial.print(heave_acc_offset);
      Serial.print(",");
      Serial.println(filt_heave_acc);
    }
    else if (now_millis - accel_init_timer > 61000 && accel_init == false ) {
      accel_init = true;
    }

    if (accel_init) {

      if (filt_heave_acc - heave_acc_offset > 0.1) {
        heave_acc =  filt_heave_acc - heave_acc_offset;
      }
      else {
        heave_acc = 0; // thresholding
      }

      heave = heave + (heave_vel * delta_t) / 1000 + 0.5 * ( heave_acc * delta_t*delta_t) / (1000 * 1000) ;
      heave_vel = heave_vel + (heave_acc * delta_t) / 1000;

      Serial.print(heave);
      Serial.print(",");
      Serial.print(heave_vel);
      Serial.print(",");
      //      Serial.print(vaccel.z());
      //      Serial.print(",");
      //      Serial.print(filt_heave_acc);
      //      Serial.print(",");
      Serial.println(filt_heave_acc - heave_acc_offset);
    }
  }
  else {
    digitalWrite(ALED_1_PIN, LOW);
    /* Display calibration status for each sensor. */

    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
