// Khai bao chan dieu khien dong co
#define F_RPWM 18
#define F_LPWM 19
#define F_RLEN 23
#define FLIMIT_SWITH 13

#define MSTEP_PULSE 26
#define MSTEP_EN 32
#define MSTEP_DIR 25
#define MLIMIT_SWITH 35

#define LSTEP_PULSE 14
#define LSTEP_EN 32
#define LSTEP_DIR 27
#define LLIMIT_SWITH 34

#define F_LEDC_CHANNEL 2
#define L_LEDC_CHANNEL 4
#define M_LEDC_CHANNEL 0

#define hut 33

void motor_init()
{   
    pinMode(F_RPWM, OUTPUT);
    pinMode(F_LPWM, OUTPUT);
    pinMode(F_RLEN, OUTPUT);

    pinMode(LSTEP_DIR, OUTPUT);
    pinMode(LSTEP_PULSE, OUTPUT);
    pinMode(LSTEP_EN, OUTPUT);

    pinMode(MSTEP_DIR, OUTPUT);
    pinMode(MSTEP_PULSE, OUTPUT);
    pinMode(MSTEP_EN, OUTPUT);

    pinMode(hut, OUTPUT);

    pinMode(FLIMIT_SWITH, INPUT_PULLUP);
    pinMode(MLIMIT_SWITH, INPUT_PULLUP);
    pinMode(LLIMIT_SWITH, INPUT_PULLUP);

    ledcSetup(M_LEDC_CHANNEL, 10000, 8);
    ledcAttachPin(MSTEP_PULSE, M_LEDC_CHANNEL);
    ledcSetup(L_LEDC_CHANNEL, 10000, 8);
    ledcAttachPin(LSTEP_PULSE, L_LEDC_CHANNEL);
    ledcSetup(F_LEDC_CHANNEL, 300, 10);
    ledcAttachPin(F_RLEN, F_LEDC_CHANNEL);

    delay(1000);
    ledcWriteTone(L_LEDC_CHANNEL, 0);
    ledcWriteTone(M_LEDC_CHANNEL, 0);
    ledcWrite(F_LEDC_CHANNEL, 0);
}

void huongDiLen(){
    digitalWrite(F_RPWM,HIGH);
    digitalWrite(F_LPWM,LOW);
}
void huongDiXuong(){
    digitalWrite(F_RPWM,LOW);
    digitalWrite(F_LPWM,HIGH);
}