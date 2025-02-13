#include <Arduino.h>

#define PIN_TX2 17
#define PIN_RX2 16

#define PIN_IN3 19
#define PIN_IN4 18
#define PIN_ENB 23

#define RL_OFF 13
#define RL_ON 14
#define RL_AL 27
#define RL4 26

#define PIN_ON 35
#define PIN_OFF 34
#define PIN_AL 32
#define PIN_RS 33

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
int pos = 0;

bool bangTai = 0;
bool old_bangTai = 0;
bool pushOn = 0;
bool pushOff = 0;
bool pushRs = 0;
bool pushAl = 0;
bool bien_alarm = 0;


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    while (Serial2.available() > 0 && newData == false) {
        rc = Serial2.read();

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
        if (receivedChars[0] == 'a') {
            bangTai = 1;
            digitalWrite(RL_ON, HIGH);
            digitalWrite(RL_OFF, LOW);
        }
        if (receivedChars[0] == 'b') {
            bangTai = 0;
            bien_alarm = 0;
            digitalWrite(RL_ON, LOW);
            digitalWrite(RL_OFF, HIGH);
            digitalWrite(RL_AL, LOW);
        }
        if(receivedChars[0] == 'r'){
            bien_alarm = 0;
            digitalWrite(RL_AL, LOW);
        }
        newData = false;
    }
}

void SerialReceiveTask(void *parameter) {
    for (;;) {
        recvWithStartEndMarkers();
        showNewData();
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

void SerialSendTask(void *parameter) {
    for (;;) {
        if(pushOn){
            Serial2.print("<a0>");
            Serial2.println();
            pushOn = 0;
        }
        if(pushOff){
            Serial2.print("<b0>");
            Serial2.println();
            pushOff = 0;
        }
        if(pushAl){
            Serial2.print("<l0>");
            Serial2.println();
            pushAl = 0;
        }
        if(pushRs){
            Serial2.print("<r0> ");
            Serial2.println();
            pushRs = 0;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
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
    4000,     /* Stack size in words */
    NULL,      /* Task input parameter */
    0,         /* Priority of the task */
    NULL,    /* Task handle. */
    1);        /* Core where the task should run */
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, PIN_RX2, PIN_TX2);
    
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);

    pinMode(RL_ON, OUTPUT);
    pinMode(RL_OFF, OUTPUT);
    pinMode(RL_AL, OUTPUT);
    pinMode(RL4, OUTPUT);

    pinMode(PIN_ON, INPUT_PULLUP);
    pinMode(PIN_OFF, INPUT_PULLUP);
    pinMode(PIN_AL, INPUT_PULLUP);
    pinMode(PIN_RS, INPUT_PULLUP);

    digitalWrite(RL_AL, LOW);
    digitalWrite(RL_ON, LOW);
    digitalWrite(RL_OFF, HIGH);

    task_s1_init();
    task_s2_init();
    
    ledcSetup(0, 50, 10);
    ledcAttachPin(PIN_ENB, 0);

    delay(50);
    ledcWrite(0, 0);

    bangTai = 0;
    pushOff = 1;
    bien_alarm = 0;
}

void loop() {
    if(bangTai != old_bangTai){
        if(bangTai){
            digitalWrite(PIN_IN3,LOW);
            digitalWrite(PIN_IN4,HIGH);
            ledcWrite(0,250);
        }
        else{
            digitalWrite(PIN_IN3,LOW);
            digitalWrite(PIN_IN4,LOW);
            ledcWrite(0,0);
        }
        old_bangTai = bangTai;
    }

    if(digitalRead(PIN_OFF)){
        if(digitalRead(PIN_AL)){
            delay(20);
            if(digitalRead(PIN_OFF)){
                pushOff = 1;
                bangTai = 0;
                bien_alarm = 0;
                digitalWrite(RL_ON, LOW);
                digitalWrite(RL_OFF, HIGH);
                digitalWrite(RL_AL, LOW);
            }
        }
    }

    if(!digitalRead(PIN_ON)){
        delay(20);
        if(!digitalRead(PIN_ON)){
            if(!bien_alarm){
                pushOn = 1;
                bangTai = 1;
                digitalWrite(RL_ON, HIGH);
                digitalWrite(RL_OFF, LOW);
            }
        }  
    }

    if(!digitalRead(PIN_AL)){
        delay(20);
        if(!digitalRead(PIN_AL)){
            pushAl = 1;
            bien_alarm = 1;
            bangTai = 0;
            digitalWrite(RL_AL, HIGH);
            digitalWrite(RL_ON, LOW);
            digitalWrite(RL_OFF, LOW);
        }
    }


    if(!digitalRead(PIN_RS)){
        if(digitalRead(PIN_AL)){
            delay(20);
            if(!digitalRead(PIN_RS)){
                pushRs = 1;
                bien_alarm = 0;
                digitalWrite(RL_AL, LOW);
            }
        }
    }
}
