
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
int pos = 0;

const byte numChars2 = 32;
char receivedChars2[numChars2];
boolean newData2 = false;
int pos2 = 0;

/// Receive 2 ////////////////////////////////////////////////////////////
void recvWithStartEndMarkers2() {
  static boolean recvInProgress2 = false;
  static byte ndx2 = 0;
  char startMarker2 = '<';
  char endMarker2 = '>';
  char rc2;
  while (Serial2.available() > 0 && newData2 == false) {
    rc2 = Serial2.read();
    if (recvInProgress2 == true) {
      if (rc2 != endMarker2) {
        receivedChars2[ndx2] = rc2;
        ndx2++;
        if (ndx2 >= numChars2) {
          ndx2 = numChars2 - 1;
        }
      } else {
        receivedChars2[ndx2] = '\0';  // terminate the string
        pos2 = atoi(&receivedChars2[1]);
        recvInProgress2 = false;
        ndx2 = 0;
        newData2 = true;
      }
    }
    else if (rc2 == startMarker2) {
      recvInProgress2 = true;
    }
  }
}

void showNewData2() {
  if (newData2 == true) {
    if (receivedChars2[0] == 'a') {
      tt_ht = 1;
      start_request = 1;
    }
    if (receivedChars2[0] == 'b') {
      tt_ht = 0;
      stop_request = 1;
      digitalWrite(hut,LOW);
    }
    if (receivedChars2[0] == 'r') {
      request_home = 1;
      request_home1 = 1;
      request_home2 = 1;
      request_home3 = 1;
      MsetHomeDone = 0;
      McontrolPID = 0;
      LsetHomeDone = 0;
      LcontrolPID = 0;
      FsetHomeDone = 0;
      FcontrolPID = 0;

      controlEnable = 0;
      choPhepGui = 1;
      kickGui = 1;
      START_CALLED = 0;
      M_CALLED = 0;
      x_f = 3000;
      y_f = 0;
      z_f = 0;
      x_0 = 3000;
      y_0 = 0;
      z_0 = 0;
      digitalWrite(hut,LOW);

      digitalWrite(LSTEP_EN,pos);
      digitalWrite(F_RLEN, !pos);
      if(pos==0){
        tt_robot = 1;
      }
      if(pos==1){
        tt_robot = 0;
      }
      tt_ht = 0;
      reset_request = 1;
    }
    if (receivedChars2[0] == 'l') {
      tt_ht = 2;
      alarm_request = 1;
      digitalWrite(hut,LOW);
      digitalWrite(LSTEP_EN, HIGH);
      digitalWrite(F_RLEN, LOW );
      tt_robot = 0;
    }
    newData2 = false;
  }
}

/// Receive 1 ////////////////////////////////////////////////////////////
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        pos = atoi(&receivedChars[1]);
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void showNewData() {
  if (newData == true) {

    // Set home
    if (receivedChars[0] == 'h') {
      request_home = 1;
      request_home1 = 1;
      request_home2 = 1;
      request_home3 = 1;
      MsetHomeDone = 0;
      McontrolPID = 0;
      LsetHomeDone = 0;
      LcontrolPID = 0;
      FsetHomeDone = 0;
      FcontrolPID = 0;

      controlEnable = 0;
      choPhepGui = 1;
      kickGui = 1;
      digitalWrite(hut,LOW);

      x_0 = x_f;
      y_0 = y_f;
      z_0 = z_f;

      x_f = 3000;
      y_f = 0;
      z_f = 0;
      TRAJECTORY_ENABLE = 0;
    }

    // Enable / disable robot
    if (receivedChars[0] == 'f') {
      digitalWrite(LSTEP_EN,pos);
      digitalWrite(F_RLEN, !pos);
      if(pos==0){
        tt_robot = 1;
      }
      if(pos==1){
        tt_robot = 0;
      }
      digitalWrite(hut,LOW);
    }

    // Runnnnnnn
    if (receivedChars[0] == 'q') {
      digitalWrite(hut,LOW);
      choPhepGui = 0;
      kickGui = 0;
      FORWARD_KINEMATIC();
      START_CALLED = 1;
      M_CALLED = 1;
    }

    // Toa do x y z
    if (receivedChars[0] == 'x') {
      x_0 = Pxx;
      diemHut[0] = pos;
      TRAJECTORY_ENABLE = 0;
    }
    if (receivedChars[0] == 'y') {
      y_0 = Pyy;
      diemHut[1] = pos + v_bangtai * 2;
      TRAJECTORY_ENABLE = 0;
    }
    if (receivedChars[0] == 'z') {
      z_0 = Pzz; 
      diemHut[2] = pos;
      TRAJECTORY_ENABLE = 0;
    }

    // Vat 
    if (receivedChars[0] == 'v') {
      vat = pos;
    }
    
    // Tinh dong hoc thuan
    if (receivedChars[0] == 'k') {
      FORWARD_KINEMATIC();
    }

    /////// 1 POINT ///////////////////////////////////
    if (receivedChars[0] == 'g') {
      choPhepGui = 0;
      kickGui = 0;
      START_CALLED = 0;
      M_CALLED = 0;
      resetnRunTrajectory(pos);
      digitalWrite(hut,LOW);
    }

    if (receivedChars[0] == 'm') {
      x_0 = x_f;
      x_f = pos;
      TRAJECTORY_ENABLE = 0;
    }
    if (receivedChars[0] == 'n') {
      y_0 = y_f;
      y_f = pos;
      TRAJECTORY_ENABLE = 0;
    }
    if (receivedChars[0] == 'p') {
      z_0 = z_f;
      z_f = pos;
      TRAJECTORY_ENABLE = 0;
    }

    /////////////////////////////////

    // Start / stop / reset
    if (receivedChars[0] == 'a') {
      tt_ht = 1;
      send_esp_start = 1;
    }
    if (receivedChars[0] == 'b') {
      tt_ht = 0;
      send_esp_stop = 1;
      digitalWrite(hut,LOW);
    }
    if (receivedChars[0] == 'r') {
      choPhepGui = 1;
      kickGui = 1;
      START_CALLED = 0;
      M_CALLED = 0;
      
      x_0 = x_f;
      y_0 = y_f;
      z_0 = z_f;

      x_f = 3000;
      y_f = 0;
      z_f = 0;
      
      tt_ht = 0;
      send_esp_reset = 1;
      digitalWrite(hut,LOW);
      resetnRunTrajectory(2);
    }

    //////////////////////////// GO POINT VIA ANGLE

    if (receivedChars[0] == 'd') {
      theta1_rec = pos;
      TRAJECTORY_ENABLE = 0;
    }
    if (receivedChars[0] == 'e') {
      theta2_rec = pos;
      TRAJECTORY_ENABLE = 0;
    }
    if (receivedChars[0] == 'j') {
      z_0 = z_f;
      z_f = pos;
      TRAJECTORY_ENABLE = 0;
    }
    if (receivedChars[0] == 'w') {
      x_0 = x_f;
      y_0 = y_f;
      FORWARD_KINEMATIC_(theta1_rec,theta2_rec,x_f,y_f);
      choPhepGui = 0;
      kickGui = 0;
      START_CALLED = 0;
      M_CALLED = 0;
      resetnRunTrajectory(pos);
      digitalWrite(hut,LOW);
    }
    newData = false;
  }
}

void SerialReceiveTask(void *parameter) {
  for (;;) {
    recvWithStartEndMarkers();
    showNewData();
    recvWithStartEndMarkers2();
    showNewData2();
  }
}

void SerialSendTask(void *parameter) {
  for (;;) {
    if(send_esp_start){
      Serial2.println("<a>");
      send_esp_start = 0;
    }
    if(send_esp_stop){
      Serial2.println("<b>");
      send_esp_stop = 0;
    }
    if(send_esp_reset){
      Serial2.println("<r>");
      send_esp_reset = 0;
    }
    if(reset_request){
      Serial.print("<e>");
      reset_request = 0;
    }
    if(start_request){
      Serial.print("<a>");
      start_request = 0;
    }
    if(stop_request){
      Serial.print("<b>");
      stop_request = 0;
    }
    if(alarm_request){
      Serial.print("<l>");
      alarm_request = 0;
    }
    if(FsetHomeDone && MsetHomeDone && LsetHomeDone){
      FORWARD_KINEMATIC();
    }
    Serial.print("<x");Serial.print(String(Pxx/10));Serial.print('>');
    Serial.print("<y");Serial.print(String(Pyy/10));Serial.print('>');
    Serial.print("<z");Serial.print(String(Pzz/10));Serial.print('>');
    if(choPhepGui){
      Serial.print("<g1>");
    }
    else{
      Serial.print("<g0>");
    }
    
    if(tt_robot){
      Serial.print("<r1>");
    }
    else{
      Serial.print("<r0>");
    }
    if(tt_ht == 1){
      Serial.print("<s1>");
    }
    else if(tt_ht == 0){
      Serial.print("<s0>");
    }
    else{
      Serial.print("<s2>");
    }
    if(kickGui){
      Serial.print("<d1>");
      kickGui = 0;
    }
    Serial.println();
    delay(20);
  }
}

void task_s1_init(){
    xTaskCreatePinnedToCore(
    SerialReceiveTask, /* Function to implement the task */
    "SerialReceiveTask",   /* Name of the task */
    4000,     /* Stack size in words */
    NULL,      /* Task input parameter */
    0,         /* Priority of the task */
    NULL,    /* Task handle. */
    0);        /* Core where the task should run */
}

void task_s2_init(){
    xTaskCreatePinnedToCore(
    SerialSendTask, /* Function to implement the task */
    "SerialSendTask",   /* Name of the task */
    2000,     /* Stack size in words */
    NULL,      /* Task input parameter */
    0,         /* Priority of the task */
    NULL,    /* Task handle. */
    0);        /* Core where the task should run */
}