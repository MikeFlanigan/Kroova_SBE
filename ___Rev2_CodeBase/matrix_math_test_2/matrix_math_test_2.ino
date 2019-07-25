#include <MatrixMath.h>

unsigned long loop_t = millis();
float dt = 0; // loop time in decimal seconds
float ctrl_signal = 0;
float Heave_acc = 0;
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


void setup() {
  Serial.begin(115200);

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
}

void loop() {
  dt = (float) (millis() - loop_t) / 1000;
  loop_t = millis();
  // ------------------------- Made up test stuff -----------------------
  ctrl_signal = (float) random(500, 1500) / 1000;
  Heave_acc = (float) (random(0, 1) - 0.5);
  // -------- End of --------- Made up test stuff -----------------------



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
  Matrix.Print((mtx_type*)x, 3, 1, "x");
  Matrix.Print((mtx_type*)y, 2, 1, "y");
  Matrix.Print((mtx_type*)P, 3, 3, "P");
  Matrix.Print((mtx_type*)P_minus, 3, 3, "P-");
  Matrix.Print((mtx_type*)K, 3, 3, "K");
  Matrix.Print((mtx_type*)F, 3, 3, "F");
  Serial.print(" sec:");
  Serial.print(dt);
  Serial.print(" ");
  Serial.print(ctrl_signal);
  Serial.print(" ");
  Serial.print(Heave_acc);
  Serial.print(" ");
  Serial.println("---------------------");
}
