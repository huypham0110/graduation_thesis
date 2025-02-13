#define M_DEFAULT_DIR 0
#define L_DEFAULT_DIR 1
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <AS5600.h>
#include <SPI.h>
AS5600 as5600_1;
AS5600 as5600_2;
AS5600 as5600_3;
#include <variable.cpp>
#include <read_enc.cpp>
#include <motor1.cpp>
#include <trajectory.cpp>
#include <serialdata.cpp>

//___________________________SETUP_________________________
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  delay(200);
  task_s1_init();
  task_s2_init();
  encoder_init();
  motor_init();
  timer_init();
  task_init();
  digitalWrite(F_RLEN,LOW);
  digitalWrite(LSTEP_EN,HIGH);
  digitalWrite(hut, LOW);
  delay(500);
}

void setpoint()
{
  if(request_home != old_request_home)
  {
    if(request_home)
    {
      digitalWrite(LSTEP_DIR,L_DEFAULT_DIR);
      ledcWriteTone(L_LEDC_CHANNEL,5000);

      digitalWrite(MSTEP_DIR,M_DEFAULT_DIR);
      ledcWriteTone(M_LEDC_CHANNEL,5000);

      huongDiLen();
      ledcWrite(F_LEDC_CHANNEL,300);
    }
    old_request_home = request_home;
  }

  if(request_home){
    if(!digitalRead(LLIMIT_SWITH))
    {
      digitalWrite(LSTEP_DIR,!L_DEFAULT_DIR);
      ledcWriteTone(L_LEDC_CHANNEL,0);
      L_SETPOINT_POS = (L_EN_RAW / L_RATIO) + 1720;
      request_home1 = 0;
    }
    if(!digitalRead(MLIMIT_SWITH))
    {
      digitalWrite(MSTEP_DIR,!M_DEFAULT_DIR);
      ledcWriteTone(M_LEDC_CHANNEL,0);
      M_SETPOINT_POS = (M_EN_RAW / M_RATIO) - 840;
      request_home2 = 0;
    }
    if(!digitalRead(FLIMIT_SWITH))
    {
      ledcWrite(F_LEDC_CHANNEL,0);
      F_SETPOINT_POS = (int32_t)((F_EN_RAW*100)/2048) - 9000;
      request_home3 = 0;
    }
    if((!request_home1)&&(!request_home2)&&(!request_home3))
    {
      request_home = 0;
      theta1 = 0;
      theta2 = 0;
      d1_new = 0;
      controlEnable = 1;
    }
  }
}

void FPID(){
  F_CONTROL_POS = d1_new*100;
  F_EP = F_CONTROL_POS - F_ENCODER_POS;
  F_ED = F_EP - F_OEP;
  F_OEP = F_EP;
  F_EI = F_ED - F_OED;
  F_OED = F_ED;
  F_PID_OUT = F_EP * F_KP - F_ED * F_KD + F_KI * F_EI;

  if (F_PID_OUT > FV_MAX) {
    F_PID_OUT = FV_MAX;
  }
  if (F_PID_OUT < -FV_MAX) {
    F_PID_OUT = -FV_MAX;
  }
  FV = (int32_t)(F_PID_OUT);

}

void F_CONTROL(){
  if(!FsetHomeDone && (abs(F_ENCODER_POS) > (100)) ){
    huongDiXuong();
    ledcWrite(F_LEDC_CHANNEL,300);  
  }
  else{
    FsetHomeDone = 1;
    FcontrolPID = 1;
  }
  if(FsetHomeDone){
    if(FV > 0){
      huongDiLen();
      ledcWrite(F_LEDC_CHANNEL,FV);
    }
    else {
      huongDiXuong();
      ledcWrite(F_LEDC_CHANNEL,-FV);
    }
  }

}

void LPID() {
  L_CONTROL_POS = (int32_t)((theta2*2048)/PI);
  L_EP = L_CONTROL_POS - L_ENCODER_POS;
  if ((L_EP < 2) && (L_EP > -2)) { L_EP = 0; }
  L_EI = L_EI + L_EP;

  if (L_EI > 10000) {
    L_EI = 10000;
  }
  if (L_EI < -10000) {
    L_EI = -10000;
  }
  L_PID_OUT = (int32_t)(L_EP * L_KP - L_ED * L_KD + L_KI * L_EI);
  if (L_PID_OUT > LV_MAX) {
    L_PID_OUT = LV_MAX;
  }
  if (L_PID_OUT < -LV_MAX) {
    L_PID_OUT = -LV_MAX;
  }
  LV = (int32_t)(L_PID_OUT);
}

void LSTEP_CONTROL() {
  if(!LsetHomeDone && (abs(L_ENCODER_POS) > (5*4096/360)) ){
      digitalWrite(LSTEP_DIR,!L_DEFAULT_DIR);
      ledcWriteTone(L_LEDC_CHANNEL,5000);
  }
  else{
    LsetHomeDone = 1;
    LcontrolPID = 1;
  }

  if(LsetHomeDone){
    if (LV > 0) {
      LDIR = 0;
      if (LDIR != OLDIR) {

        OLDIR = LDIR;
        digitalWrite(LSTEP_DIR, LOW);
      }
      ledcWriteTone(L_LEDC_CHANNEL, LV);
    } 
    else {
      LDIR = 1;
      if (LDIR != OLDIR) {
        OLDIR = LDIR;
        digitalWrite(LSTEP_DIR, HIGH);
      }
      ledcWriteTone(L_LEDC_CHANNEL, -LV);
    }
  }
}



void MPID() {
  M_CONTROL_POS =  (int32_t)((theta1*2048)/PI);
  M_EP = M_CONTROL_POS - M_ENCODER_POS;
  M_ED = M_EP - M_OEP;
  M_OEP = M_EP;
  if ((M_EP < 1) && (M_EP > -1)) { M_EP = 0; }
  M_EI = M_EI + M_EP;
  if (M_EI > 10000) {
    M_EI = 10000;
  }
  if (M_EI < -10000) {
    M_EI = -10000;
  }

  M_PID_OUT = (int32_t)(M_EP * M_KP - M_ED * M_KD + M_KI * M_EI);
  if (M_PID_OUT > MV_MAX) {
    M_PID_OUT = MV_MAX;
  }
  if (M_PID_OUT < -MV_MAX) {
    M_PID_OUT = -MV_MAX;
  }
  MV = (int32_t)(M_PID_OUT);
}

void MSTEP_CONTROL() {
  if(!MsetHomeDone && (abs(M_ENCODER_POS) > (5*4096/360)) ){
      digitalWrite(MSTEP_DIR,!M_DEFAULT_DIR);
      ledcWriteTone(M_LEDC_CHANNEL,5000);
  }
  else{
    MsetHomeDone = 1;
    McontrolPID = 1;
  }

  if(MsetHomeDone){
    if (MV > 0) {
      MDIR = 0;
      if (MDIR != OMDIR) {

        OMDIR = MDIR;
        digitalWrite(MSTEP_DIR, LOW);
      }
      ledcWriteTone(M_LEDC_CHANNEL, MV);
    } 
    else {
      MDIR = 1;
      if (MDIR != OMDIR) {
        OMDIR = MDIR;
        digitalWrite(MSTEP_DIR, HIGH);
      }
      ledcWriteTone(M_LEDC_CHANNEL, -MV);
    }
  }
}


void TWO_POINT_CONTROL() {
  if ((M_CALLED == 1) && (!TRAJECTORY_ENABLE)) {
    kickGui = 0;
    choPhepGui = 0;
    digitalWrite(hut,LOW);
    x_f = diemHut[0];
    y_f = diemHut[1];
    z_f = z_f;
    resetnRunTrajectory(1);
  }
  if ((M_CALLED == 2) && (!TRAJECTORY_ENABLE)) {

    bienHut = 1;
    x_f = x_f;
    y_f = y_f;
    z_f = diemHut[2];
    resetnRunTrajectory(1);
  }
  if ((M_CALLED == 3) && (!TRAJECTORY_ENABLE)) {
    x_f = x_f;
    y_f = y_f;
    z_f = 0;
    resetnRunTrajectory(1);
  }
  if ((M_CALLED == 4) && (!TRAJECTORY_ENABLE)) {
    if(vat==0){
      x_f = diemTha1[0];
      y_f = diemTha1[1];
      z_f = 0;
      resetnRunTrajectory(2);
    }
    if(vat==1){
      x_f = diemTha2[0];
      y_f = diemTha2[1];
      z_f = 0;
      resetnRunTrajectory(2);
    }
  }
  if ((M_CALLED == 5) && (!TRAJECTORY_ENABLE)) {
    digitalWrite(hut,LOW);
    choPhepGui = 1;
    kickGui = 1;
    x_f = diemHome[0];
    y_f = diemHome[1];
    z_f = diemHome[2];
    resetnRunTrajectory(1.5);  
  }
  if (M_CALLED == 6) {
    START_CALLED = 0;
    M_CALLED = 0;
  }
}

void run(){
  if(controlEnable){
    if(LcontrolPID){
      LPID();
    }
    if(McontrolPID){
      MPID();
    }
    if(FcontrolPID){
      FPID();
    }
    F_CONTROL();
    LSTEP_CONTROL();
    MSTEP_CONTROL();
    TWO_POINT_CONTROL();
  }
}

void loop() {
  encoder_read();
  setpoint();
  run();
  TRAJECTORY_CONTROL(); 
}
//----------------------------END---------------------------