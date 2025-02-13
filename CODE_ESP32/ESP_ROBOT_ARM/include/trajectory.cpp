#define SAMPLE_TIME 100

hw_timer_t *Timer0_Cfg = NULL;
TaskHandle_t Task1;

BLA::Matrix<4, 4> Y_x;
BLA::Matrix<4, 4> A_x;
BLA::Matrix<4, 4> Y_y;
BLA::Matrix<4, 4> Y_z;
BLA::Matrix<4, 3> X;
BLA::Matrix<4, 4> A_inv;
BLA::Matrix<4, 4> X_x;
BLA::Matrix<4, 4> X_y;
BLA::Matrix<4, 4> X_z;

void FORWARD_KINEMATIC_(double theta1_rec, double theta2_rec, double& Pxx, double& Pyy) {
  Pxx = (a1 + a3 * cos(((theta1_rec + theta2_rec) * M_PI) / 180) + a2 * cos((theta1_rec * M_PI) / 180)) * 10;
  Pyy = (a3 * sin(((theta1_rec + theta2_rec) * M_PI) / 180) + a2 * sin(((theta1_rec)*M_PI) / 180)) * 10;
}

void FORWARD_KINEMATIC() {
  etheta1 = (double)((((int32_t)(M_ENCODER_POS)) * 360) / 4096);
  etheta2 = (double)((((int32_t)(L_ENCODER_POS)) * 360) / 4096);
  Pxx = (a1 + a3 * cos(((etheta1 + etheta2) * M_PI) / 180) + a2 * cos((etheta1 * M_PI) / 180)) * 10;
  Pyy = (a3 * sin(((etheta1 + etheta2) * M_PI) / 180) + a2 * sin(((etheta1)*M_PI) / 180)) * 10;
  Pzz = (d1_new - d2 - d3 - d4) * 10;
}

void INVERSE_KINEMATIC(double xq, double yq, double zq){
  d_inv = (zq / 10) + d2 + d3 + d4;
  c2 = (((xq / 10) - a1) * ((xq / 10) - a1) + (yq / 10) * (yq / 10) - a2 * a2 - a3 * a3) / (2 * a2 * a3);
  kc = 1 - c2 * c2;
  if (kc < 0) { kc = -kc; }
  s2 = sqrt(kc);
  theta2_inv = (double)(atan2(s2, c2));
  c1 = ((a3 * cos(theta2) + a2) * ((xq / 10) - a1) + (a3 * sin(theta2)) * (yq / 10)) / ((a3 * cos(theta2) + a2) * (a3 * cos(theta2) + a2) + (a3 * sin(theta2)) * (a3 * sin(theta2)));
  s1 = ((a3 * cos(theta2) + a2) * (yq / 10) - (a3 * sin(theta2)) * ((xq / 10) - a1)) / ((a3 * cos(theta2) + a2) * (a3 * cos(theta2) + a2) + (a3 * sin(theta2)) * (a3 * sin(theta2)));
  theta1_inv = (double)(atan2(s1, c1));
}

void Task1code(void *parameter) {
  for (;;) {
    d1_new = (zq / 10) + d2 + d3 + d4;
    c2 = (((xq / 10) - a1) * ((xq / 10) - a1) + (yq / 10) * (yq / 10) - a2 * a2 - a3 * a3) / (2 * a2 * a3);
    kc = 1 - c2 * c2;
    if (kc < 0) { 
      kc = -kc; 
    }

    if(vat==0){
      s2 = -sqrt(kc);
      theta2 = (double)(atan2(s2, c2));
      c1 = ((a3 * cos(theta2) + a2) * ((xq / 10) - a1) + (a3 * sin(theta2)) * (yq / 10)) / ((a3 * cos(theta2) + a2) * (a3 * cos(theta2) + a2) + (a3 * sin(theta2)) * (a3 * sin(theta2)));
      s1 = ((a3 * cos(theta2) + a2) * (yq / 10) - (a3 * sin(theta2)) * ((xq / 10) - a1)) / ((a3 * cos(theta2) + a2) * (a3 * cos(theta2) + a2) + (a3 * sin(theta2)) * (a3 * sin(theta2)));
      theta1 = (double)(atan2(s1, c1));
    }
    else{
      s2 = sqrt(kc);
      theta2 = (double)(atan2(s2, c2));
      c1 = ((a3 * cos(theta2) + a2) * ((xq / 10) - a1) + (a3 * sin(theta2)) * (yq / 10)) / ((a3 * cos(theta2) + a2) * (a3 * cos(theta2) + a2) + (a3 * sin(theta2)) * (a3 * sin(theta2)));
      s1 = ((a3 * cos(theta2) + a2) * (yq / 10) - (a3 * sin(theta2)) * ((xq / 10) - a1)) / ((a3 * cos(theta2) + a2) * (a3 * cos(theta2) + a2) + (a3 * sin(theta2)) * (a3 * sin(theta2)));
      theta1 = (double)(atan2(s1, c1));
    }
  }
}

void task_init(){
    xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1",   /* Name of the task */
    10000,     /* Stack size in words */
    NULL,      /* Task input parameter */
    0,         /* Priority of the task */
    &Task1,    /* Task handle. */
    0);        /* Core where the task should run */
}

void IRAM_ATTR Timer0_ISR() {
  if (controlEnable) {
    if (t < t_f) {
      t += 0.0001;
      if(M_CALLED == 2 && t > (t_f/2) && bienHut ==1){
        digitalWrite(hut,HIGH);
        bienHut =0;
      }
    }
    else{
      if(TRAJECTORY_ENABLE){
        if(START_CALLED){
          M_CALLED++;
        }
        TRAJECTORY_ENABLE = 0;
        x_0 = x_f;
        y_0 = y_f;
        z_0 = z_f;
      }
    }
  }
}

void timer_init() {
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, SAMPLE_TIME, true);
  timerAlarmEnable(Timer0_Cfg);
}

void MATRIX_CALCULATE() {
  Y_x = { x_0, 0, 0, 0, vx0, 0, 0, 0, x_f, 0, 0, 0, vxf, 0, 0, 0 };
  Y_y = { y_0, 0, 0, 0, vy0, 0, 0, 0, y_f, 0, 0, 0, vyf, 0, 0, 0 };
  Y_z = { z_0, 0, 0, 0, vz0, 0, 0, 0, z_f, 0, 0, 0, vzf, 0, 0, 0 };
  A_x = { 1, t_0, t_0 * t_0, t_0 * t_0 * t_0,
          0, 1, 2 * t_0, 3 * t_0 * t_0,
          1, t_f, t_f * t_f, t_f * t_f * t_f,
          0, 1, 2 * t_f, 3 * t_f * t_f };
  A_inv = Inverse(A_x);
  X_x = A_inv * Y_x;
  X_y = A_inv * Y_y;
  X_z = A_inv * Y_z;
  a0_x = X_x(0);
  a1_x = X_x(1);
  a2_x = X_x(2);
  a3_x = X_x(3);
  a0_y = X_y(0);
  a1_y = X_y(1);
  a2_y = X_y(2);
  a3_y = X_y(3);
  a0_z = X_z(0);
  a1_z = X_z(1);
  a2_z = X_z(2);
  a3_z = X_z(3);
  MATRIX_CALC_DONE = 1;
}

void TRAJECTORY_PLANNING() {
  xq = a0_x + a1_x * t + a2_x * t * t + a3_x * t * t * t;
  yq = a0_y + a1_y * t + a2_y * t * t + a3_y * t * t * t;
  zq = a0_z + a1_z * t + a2_z * t * t + a3_z * t * t * t;
  vx = a1_x + 2 * a2_x * t + 3 * a3_x * t * t;
  vy = a1_y + 2 * a2_y * t + 3 * a3_y * t * t;
  vz = a1_z + 2 * a2_z * t + 3 * a3_z * t * t;
}

void TRAJECTORY_CONTROL() {
  if (!MATRIX_CALC_DONE) {
    MATRIX_CALCULATE();
  }
  if (TRAJECTORY_ENABLE && MATRIX_CALC_DONE) {
    TRAJECTORY_PLANNING();
  }
}

void resetnRunTrajectory(double tg_chay){
  t_0 = 0;
  t = 0;
  t_f = tg_chay;
  TRAJECTORY_ENABLE = 1;
  MATRIX_CALC_DONE = 0;
  INVERSE_KINEMATIC(x_f,y_f,z_f);
}