#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <MPMutex.h>
MPMutex mutex(MP_MUTEX_ID0);

const int R0 = 0; // pwm0 - 6 pin
const int R1 = 1; // pwm1 - 5 pin
const int L0 = 2; // pwm2 - 9 pin
const int L1 = 3; // pwm3 - 3 pin

#define R_EN 10
#define L_EN 11
#define DELAY_TIME 40
#define MOTOR_SW 12

#define TORQUE_COMP_POWER 255
#define TORQUE_COMP_TIME_R 5
#define TORQUE_COMP_TIME_L 5

#define KP 0
#define KI 0
#define KD 0
 
float R_Kp = KP; 
float R_Ki = KI;
float R_Kd = KD;
float L_Kp = KP;
float L_Ki = KI; 
float L_Kd = KD;
float VRt = 0.0;
float VLt = 0.0;

int32_t R_duty = 0;
int32_t L_duty = 0;

const int recv_buf_size = 32;
char recv_buf[recv_buf_size] = {0};
bool param_updated = false;
  
volatile uint32_t R = 0;
volatile uint32_t L = 0;
void Encoder0() {  ++R; }
void Encoder1() {  ++L; }

struct pid_data {
  float rKp;
  float rKi;
  float rKd;
  float lKp;
  float lKi;
  float lKd;
  float vt;
  float rot;
};

struct torque_data {
  int R_duty;
  int L_duty;
};

const int subcore = 1;

struct pid_data* pid;
struct torque_data td;

float calc_speed(uint32_t enc_count, uint32_t duration_ms, float* mileage, float target) {
  static const float D  = 0.0325; // tire radius (m)
  static const float Enc_Theta = 9.0;  // a unit degree of the edge encoder 
  if (duration_ms == 0.0) { *mileage = 0.0; return 0.0; } 
  float rotation_ = enc_count*Enc_Theta;  // rotation (degree)
  float rot_radian_ = PI*rotation_/180.0; // convert dgree to radian
  float mileage_ = rot_radian_*D; // (m)
  float rov_speed_ = mileage_*1000.0/(float)(duration_ms); 
  //MPLog("%d,%f,%f,%f\n", enc_count, rotation_, mileage_, rov_speed_);
  *mileage = mileage_;
  if (target < 0) rov_speed_ = -rov_speed_; 
  return rov_speed_;
}

void setup() {
  Serial.begin(115200);
    
  pinMode(MOTOR_SW, OUTPUT);
  digitalWrite(MOTOR_SW, LOW);
  attachInterrupt(R_EN, Encoder0, CHANGE);
  attachInterrupt(L_EN, Encoder1, CHANGE);

  // setup pwm pins
  setup_pwm(0);
  setup_pwm(1);
  setup_pwm(2);
  setup_pwm(3);

  MP.begin(subcore);
  MP.RecvTimeout(MP_RECV_POLLING);

  // motor power on
  digitalWrite(MOTOR_SW, HIGH);

}

void loop() {
  static uint32_t start_time = 0;
  static uint32_t last_time = 0;
  static float R_last_err = 0.0;
  static float L_last_err = 0.0;
  static float R_err_integ = 0.0;
  static float L_err_integ = 0.0; 
  static const float d = 0.065; /* (the legnth of chassis)/2 (m) */ 

  float R_Vm, L_Vm;  // actual speeds calcurated by encorders

  static const bool debug = false;

  int8_t recv_id;
  int ret = MP.Recv(&recv_id, &pid, subcore);
  if (ret > 0) {
    R_Kp = pid->rKp; 
    R_Ki = pid->rKi;
    R_Kd = pid->rKd;
    L_Kp = pid->lKp;
    L_Ki = pid->lKi; 
    L_Kd = pid->rKd;
    if (pid->rot == 0.0) {
      VRt = VLt = pid->vt;
    } else if (pid->rot > 0.0) {
      VRt = +pid->vt;
      VLt = -pid->vt;
    } else if (pid->rot < 0.0) {
      VRt = -pid->vt;
      VLt = +pid->vt;
    }
    MPLog("param(orig) = %f,%f,%f,%f,%f,%f,%f,%f\n",
      pid->rKp,pid->rKi,pid->rKd,pid->lKp,pid->lKi,pid->rKd,pid->vt, pid->rot); 
    MPLog("param(copy) = %f,%f,%f,%f,%f,%f,%f,%f\n",
      R_Kp,R_Ki,R_Kd,L_Kp,L_Ki,L_Kd,VRt,VLt); 
    start_time = millis();
  }
 
  uint32_t current_time = millis();
  uint32_t duration = current_time - last_time; 
  last_time = current_time;    

  float R_err  = 0.0;
  float L_err  = 0.0;
  float R_derr = 0.0;
  float L_derr = 0.0;

  if (current_time - start_time > 1000) {
    // MPLog("Stop TurtleBotSpr\n"); 
    pwm_control(0, 0); // backward
    pwm_control(1, 0); // forward
    pwm_control(2, 0); // backward
    pwm_control(3, 0); // forward
    VRt = VLt = 0.0;   
    R_Vm = L_Vm = 0.0;
    R_duty = L_duty = 0;
    R_last_err = L_last_err = 0.0;
    R_err_integ = L_err_integ = 0.0;
    delay(DELAY_TIME);
    return;
  }

  if (debug) {
    MPLog("R_err_integ: %f, L_err_integ: %f, R_last_err: %f, L_last_err: %f\n",
      R_err_integ, L_err_integ, R_last_err, L_last_err);
  }

  noInterrupts();
  uint32_t cur_R = R; R = 0; 
  uint32_t cur_L = L; L = 0;
  interrupts();
  float R_mileage = 0.0;
  float L_mileage = 0.0;
  
  R_Vm = calc_speed(cur_R, duration, &R_mileage, VRt);
  L_Vm = calc_speed(cur_L, duration, &L_mileage, VLt); 
  if (debug) {
    MPLog("%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\n",
      duration,cur_R,cur_L,VRt,VLt,R_Vm,L_Vm,R_mileage,L_mileage,VRt,VLt); 
  }

  float duration_sec = duration/1000.0;
  R_err        = VRt - R_Vm;
  L_err        = VLt - L_Vm;
  R_err_integ += (R_err + R_last_err)*0.5*duration_sec;
  L_err_integ += (L_err + L_last_err)*0.5*duration_sec;
  R_derr       = (R_err - R_last_err)/duration_sec;  
  L_derr       = (L_err - L_last_err)/duration_sec;

  if (debug) {
    MPLog("R_Kp: %f, R_Ki: %f, R_Kd: %f,L_Kp: %f, L_Ki: %f, L_Kd: %f\n",
        R_Kp, R_Ki, R_Kd, L_Kp, L_Ki, L_Kd);
    MPLog("R_err: %f, L_err: %f, R_err_integ: %f, L_err_integ: %f, R_derr: %f, L_derr: %f\n",
        R_err, L_err, R_err_integ, L_err_integ, R_derr, L_derr);
  }


  R_duty = (int32_t)(R_Kp*R_err + R_Ki*R_err_integ + R_Kd*R_derr);
  L_duty = (int32_t)(L_Kp*L_err + L_Ki*L_err_integ + L_Kd*L_derr);

  if (abs(R_duty) > 255) {
    if (debug) MPLog("Over range on R: %d\n", R_duty);
    if (R_duty > 255)    R_duty = +255;
    else if (R_duty < 0) R_duty = -255;
  }

  if (abs(L_duty) > 255) {
    if (debug) MPLog("Over range on L: %d\n", L_duty);
    if (L_duty > 255)    L_duty = +255;
    else if (L_duty < 0) L_duty = -255;
  }

  R_last_err = R_err;
  L_last_err = L_err;   

  MPLog("R_duty: %d, L_duty: %d\n", R_duty, L_duty);
  td.R_duty = R_duty;
  td.L_duty = L_duty;
  int8_t snd_id = 101;
  MP.Send(snd_id, &td, subcore);
  
  if (R_Vm == 0.0 && VRt != 0.0) {
    // Torque Compensation
    if (VRt > 0.0) {
      pwm_control(0, 0);                 // backward
      pwm_control(1, TORQUE_COMP_POWER); // forward
    } else if (VRt < 0.0) {
      pwm_control(1, TORQUE_COMP_POWER); // backward
      pwm_control(0, 0);                 // forward
    }
    delay(TORQUE_COMP_TIME_R);
  };
  
  if (VRt > 0.0) {
    pwm_control(0, 0);           // backward
    pwm_control(1, abs(R_duty)); // forward
  } else if (VRt < 0.0) {
    pwm_control(0, abs(R_duty)); // backward
    pwm_control(1, 0);           // forward
  } else if (VRt == 0.0) {
    pwm_control(0, 0);           // backward
    pwm_control(1, 0);           // forward
  }

  if (L_Vm == 0.0 && VLt != 0.0) {
    // Torque Compensation
    if (VRt > 0.0) {
      pwm_control(2, 0);                 // backward
      pwm_control(3, TORQUE_COMP_POWER); // forward
    } else if (VRt < 0.0) {
      pwm_control(2, TORQUE_COMP_POWER); // backward
      pwm_control(3, 0);                 // forward
    }
    delay(TORQUE_COMP_TIME_L);
  };  

  if (VLt > 0.0) {
    pwm_control(2, 0);           // backward
    pwm_control(3, abs(L_duty)); // forward
  } else if (VLt < 0.0) {
    pwm_control(2, abs(L_duty)); // backward
    pwm_control(3, 0);           // forward    
  } else if (VLt == 0.0) {
    pwm_control(2, 0);           // backward
    pwm_control(3, 0);           // forward     
  }

  delay(DELAY_TIME);
}
