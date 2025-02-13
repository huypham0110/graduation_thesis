
double L_RATIO = 3.2;
#define M_RATIO 40
#define F_RATIO 1

int32_t F_EN_RAW = 0;
int32_t M_EN_RAW = 0;
int32_t L_EN_RAW = 0;

int32_t F_ENCODER_POS = 0;
int32_t M_ENCODER_POS = 0;
int32_t L_ENCODER_POS = 0;

double F_SETPOINT_POS = 0;
double M_SETPOINT_POS = 0;
double L_SETPOINT_POS = 0;

bool FDIR = 0;
bool OFDIR = 0;
bool LDIR = 0;
bool OLDIR = 0;
bool MDIR = 0;
bool OMDIR = 0;

bool request_home = 0;
bool request_home1 = 0;
bool request_home2 = 0;
bool request_home3 = 0;
bool old_request_home = 0;
bool controlEnable = 0;

bool FcontrolPID = 0;
bool McontrolPID = 0;
bool LcontrolPID = 0;

bool FsetHomeDone = 0;
bool MsetHomeDone = 0;
bool LsetHomeDone = 0;

double theta2 = 0;
double theta1 = 0;

double theta1_inv = 0;
double theta2_inv = 0;
double d_inv = 0;

/// F PID ///
int32_t F_CONTROL_POS = 0;
int32_t F_EP = 0;
int32_t F_OEP = 0;
int32_t F_EI = 0;
int32_t F_OEI = 0;
int32_t F_ED = 0;
int32_t F_OED = 0;

int32_t F_KP = 3;
double F_KI = 0.1;
int32_t F_KD = 0;

int32_t F_PID_OUT = 0;
int32_t FV_MAX= 1024;
int32_t FV = 0;

/// L PID ///
int32_t L_CONTROL_POS = 0;
int32_t L_EP = 0;
int32_t L_EI = 0;
int32_t L_ED = 0;

int32_t L_KP = 20;
int32_t L_KI = 0;
int32_t L_KD = 0;

int32_t L_PID_OUT = 0;
int32_t LV_MAX= 10000;
int32_t LV = 0;

/// M PID ///
int32_t M_CONTROL_POS = 0;
int32_t M_EP = 0;
int32_t M_OEP = 0;
int32_t M_EI = 0;
int32_t M_ED = 0;

int32_t M_KP = 100;
int32_t M_KI = 0;
int32_t M_KD = 0;

int32_t M_PID_OUT = 0;
int32_t MV_MAX= 10000;
int32_t MV = 0;

///////////////////////////////////////////////trajetory

double d1_new = 0;
double c2 = 0;
double s2 = 0;
double c1 = 0;
double s1 = 0;

double a1 = 0;
double a2 = 146;
double a3 = 154;
double d2 = 0;
double d3 = 0;
double d4 = 0;

bool TRAJECTORY_ENABLE = 0;
bool MATRIX_CALC_DONE = 1;

double a0_x = 0;
double a1_x = 0;
double a2_x = 0;
double a3_x = 0;
double a0_y = 0;
double a1_y = 0;
double a2_y = 0;
double a3_y = 0;
double a0_z = 0;
double a1_z = 0;
double a2_z = 0;
double a3_z = 0;

double t_0 = 0;
double x_0 = 3000;
double y_0 = 0;
double z_0 = 0;
double vx0 = 0;
double vy0 = 0;
double vz0 = 0;

double t = 0;
double xq = 3000;
double yq = 0;
double zq = 0;
double vx = 0;
double vy = 0;
double vz = 0;


double Pxx = 0;
double Pyy = 0;
double Pzz = 0;
double etheta1 = 0;
double etheta2 = 0;

double t_f = 0;
double t_interval = 3;

double x_m1 = 0;
double y_m1 = 0;
double z_m1 = 0;

double x_m2 = 0;
double y_m2 = 0;
double z_m2 = 0;

double x_m3 = 0;
double y_m3 = 0;
double z_m3 = 0;

double x_m4 = 0;
double y_m4 = 0;
double z_m4 = 0;

double x_m5 = 0;
double y_m5 = 0;
double z_m5 = 0;

double x_m6 = 0;
double y_m6 = 0;
double z_m6 = 0;

double x_f = 3000;
double y_f = 0;
double z_f = 0;
double vxf = 0;
double vyf = 0;
double vzf = 0;
double kc = 0;
double OLV = 0;

//////////////////////
double theta1_speed = 0;
double h = 0;
double otheta1 = 0;
double START_CALLED = 0;
int k = 0;
double M_CALLED = 0;
int vat =0;
double diemTha1[3]={400,-1800,0};
double diemTha2[3]={400,1800,0};
double diemHome[3]={3000,0,0}; 
double diemHut[3]={3000,0,-800};
bool choPhepGui = 0;
double v_bangtai = 960;
double i=0;
bool tt_robot = 0;
int8_t tt_ht = 0;
bool send_esp_start = 0;
bool send_esp_stop = 0;
bool send_esp_reset = 0;
bool reset_request = 0;
bool start_request = 0;
bool stop_request = 0;
bool alarm_request = 0;
bool choPhepHut = 0;
bool oldChoPhepHut = 1;
double t_v = 2;
double theta1_rec = 0;
double theta2_rec = 0;
bool kickGui = 0;
bool bienHut = 0;