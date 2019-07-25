#include <BasicLinearAlgebra.h>

using namespace BLA;

unsigned long loop_t = millis();
int dt = 0; // loop time in decimal seconds
int ctrl_signal = 0;
int Heave_acc = 0;
bool first_loop_flag = true;





void setup() {
  Serial.begin(115200);



  // ------------------------- Kalman Filter Matrix Math Setup -----------------------





  //  // Covariance matrix
  //  BLA::Matrix<3, 3> P = {0.2, 0, 0,
  //                         0, 0.2, 0,
  //                         0, 0, 0.02
  //                        };
  //  // Predicted covariance matrix
  //  BLA::Matrix<3, 3> P_minus;
  //  //P_minus.Fill(0);
  //
  //  // State vector
  //  BLA::Matrix<3, 1> x = {0, 0, 0};    // RH, RH_v, RH_acc
  //  //x.Fill(0);
  //
  //  // Predicted state vector
  //  BLA::Matrix<3, 1> x_minus = {0, 0, 0};
  //  //x_minus.Fill(0);
  //
  //  // Kalman gain
  //  BLA::Matrix<3, 2> K = {0, 0, 0};
  //  //K.Fill(0);
  //
  //  // Measurement vector
  //  BLA::Matrix<2, 1> y = {0, 0, 0};
  //  //y.Fill(0);
  //  // -------- End of --------- Kalman Filter Matrix Math Setup -----------------------



  //  x << ctrl_signal / 1000,
  //  0.0,
  //  Heave_acc;
  //  Serial << x;

}
int count = 0;
void loop() {
  if (first_loop_flag) {
    //    BLA::Matrix<3, 3> I = {1, 0, 0, 0, 1, 0, 0, 0, 1};                  // 3x3 identity matrix
    //    BLA::Matrix<3, 3> F;                                                // State transition matrix
    //    BLA::Matrix<2, 3> H = {1, 0, 0, 0, 0, 1};                           // Measurement matrix
    //    BLA::Matrix<3, 3> Qkf = {0.1, 0, 0, 0, 0.01, 0, 0, 0, 0.01};        // Process noise
    //    BLA::Matrix<2, 2> R = {0.1, 0, 0, 0.01};                            // Measurement noise
    //    BLA::Matrix<3, 3> P;   // Covariance matrix
    //    BLA::Matrix<3, 3> P_minus; // Predicted covariance matrix
    //    BLA::Matrix<3, 1> x;   // State vector
    //    BLA::Matrix<3, 1> x_minus; // Predicted state vector
    //    BLA::Matrix<3, 2> K;   // Kalman gain
    //    BLA::Matrix<2, 1> y; // Measurement vector


  }

    Matrix<3, 3> I = {1, 0, 0, 0, 1, 0, 0, 0, 1};                  // 3x3 identity matrix
    Matrix<3, 3> F;                                                // State transition matrix
    Matrix<2, 3> H = {1, 0, 0, 0, 0, 1};                           // Measurement matrix
    Matrix<3, 3> Qkf = {0.1, 0, 0, 0, 0.01, 0, 0, 0, 0.01};        // Process noise
    Matrix<2, 2> R = {0.1, 0, 0, 0.01};                            // Measurement noise
    Matrix<3, 3> P;   // Covariance matrix
    Matrix<3, 3> P_minus; // Predicted covariance matrix
    Matrix<3, 1> x;   // State vector
    Matrix<3, 1> x_minus; // Predicted state vector
    Matrix<3, 2> K;   // Kalman gain
    Matrix<2, 1> y; // Measurement vector

  dt = (millis() - loop_t);
  loop_t = millis();
  // ------------------------- Made up test stuff -----------------------
  ctrl_signal = random(500, 1500); // !!!!!! / 1000;
  Heave_acc = (random(1000, 2000) - 2000);// / 10;
  // -------- End of --------- Made up test stuff -----------------------



  // only begin Kalman filter once get readings from US and RH and initialize the matrices!!!!
  if (first_loop_flag) { // collect data and initialize KF variables
    first_loop_flag = false;
    x << ctrl_signal, 0, Heave_acc;
  }
  else { // variables have been initialized on the first loop

    //    Serial.print(dt);
    //    Serial.print(" ");
    //    Serial << x;
    //    Serial.print(" ");
    //    Serial.println(" ");


    // recompute F since it's time dependent
    F << 1, dt, 0.5 * dt*dt,
    0, 1, dt,
    0, 0, 1;

    // Perform prediction step
    //    Multiply(F, x, x_minus);
    x_minus = F * x;
    //        P_minus << F*P * ~F + Qkf;
    //
    //        // Compute Kalman gain
    //        K << P_minus * ~H / (H * P_minus * ~H + R);
    //
    //        // gather measurement vector
    //        y <<  ctrl_signal,
    //        Heaveacc; // watch out for mm meters
    //
    //        // Prediction correction step
    //        x << x_minus + K*(y - H * x_minus);
    //        P << (I - K * H)*P_minus;
  }
  if (count < 10) {
    Serial << x;
    Serial.println(" ");
  }
  count += 1;
}
