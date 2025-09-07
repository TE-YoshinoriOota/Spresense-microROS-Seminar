#ifndef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>

/* Use CMSIS library */
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <arm_math.h>

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

#define MOTOR_POWER_MSG 100
#define COMMAND_MSG     101
#define REQ_ODOM        102

float R_Kp = 120; 
float R_Ki =  30;
float R_Kd =   3;
float L_Kp = 120;
float L_Ki =  30; 
float L_Kd =   3;
float VRt =  0.0;
float VLt =  0.0;

int32_t R_duty = 0;
int32_t L_duty = 0;

struct rover_odm {
  float odm_ang_z;
  float odm_pos_x;
  float odm_pos_y;
  float odm_qt_qz;
  float odm_qt_qw;
};

struct rover_odm rover_odm;

struct qua {
  float qx;
  float qy;
  float qz;
  float qw;
};

// odometry pose
static float odm_lin_x = 0.0;
static float odm_ang_z = 0.0;
static float odm_pos_x = 0.0;
static float odm_pos_y = 0.0;
static float odm_pos_z = 0.036;
// odometry quaternion
static float odm_qt_qx = 0.0;
static float odm_qt_qy = 0.0;
static float odm_qt_qz = 0.0;
static float odm_qt_qw = 0.0;

volatile uint32_t R = 0;
volatile uint32_t L = 0;
void Encoder0() {  ++R; }
void Encoder1() {  ++L; }


float calc_speed(uint32_t enc_count, uint32_t duration_ms, float* mileage, float target) {
  static const float r  = 0.0325; // tire radius (m)
  static const float enc_theta = 9.0;  // a unit degree of the edge encoder 
  if (duration_ms == 0.0) { *mileage = 0.0; return 0.0; }
  float rotation_ = enc_count*enc_theta; // rotation (degree) 
  float mileage_ = PI*r*rotation_/180.0; // convert degree to radian
  float rov_speed_ = mileage_*1000.0/(float)(duration_ms); 
  //Serial.print(String(enc_count) + "," + String(rotation, 6) + "," + String(mileage, 6) + "," + String(rov_speed) + ",");
  *mileage = mileage_;
  if (target < 0) rov_speed_ = -rov_speed_;
  return rov_speed_;
}

void quaternion_from_euler(float roll, float pitch, float yaw, struct qua* q) {
  float cr = arm_cos_f32(roll * 0.5f);
  float sr = arm_sin_f32(roll * 0.5f);
  float cp = arm_cos_f32(pitch * 0.5f);
  float sp = arm_sin_f32(pitch * 0.5f);
  float cy = arm_cos_f32(yaw * 0.5f);
  float sy = arm_sin_f32(yaw * 0.5f);

  q->qw = cr * cp * cy + sr * sp * sy;
  q->qx = sr * cp * cy - cr * sp * sy;
  q->qy = cr * sp * cy + sr * cp * sy;
  q->qz = cr * cp * sy - sr * sp * cy;
}

void update_odometry(float curr_R_vel, float curr_L_vel, float duration) {
  // calc pose
  static float pos_x = 0.0;
  static float pos_y = 0.0;
  static float ang_z = 0.0;

  float v = (curr_R_vel + curr_L_vel) / 2.;
  float delta_x = v*duration*arm_cos_f32(ang_z);
  float delta_y = v*duration*arm_sin_f32(ang_z);

  //float last_ang_z = ang_z;
  float omega = (curr_R_vel - curr_L_vel) / (2.*d - 0.01);  // 0.01 is adjustment valueIM
  float delta_angle = omega * duration;

  pos_x += delta_x;
  pos_y += delta_y;
  ang_z += delta_angle;
  if (ang_z >=  2.*M_PI) ang_z -= 2.*M_PI;
  if (ang_z <= -2.*M_PI) ang_z += 2.*M_PI;

  // calc quaternion
  struct qua q;
  quaternion_from_euler(0., 0., ang_z, &q);

  odm_lin_x = v;
  odm_ang_z = omega;
  odm_pos_x = pos_x;
  odm_pos_y = pos_y;
  odm_qt_qx = q.qx;
  odm_qt_qy = q.qy;
  odm_qt_qz = q.qz;
  odm_qt_qw = q.qw;
}

void setup() {
  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING);
    
  pinMode(MOTOR_SW, OUTPUT);
  digitalWrite(MOTOR_SW, LOW);
  attachInterrupt(R_EN, Encoder0, CHANGE);
  attachInterrupt(L_EN, Encoder1, CHANGE);

  // setup pwm pins
  setup_pwm(0);
  setup_pwm(1);
  setup_pwm(2);
  setup_pwm(3);

  memset(&rover_odm, 0, sizeof(struct rover_odm));
}

void loop() {
  static uint32_t start_time = 0;
  static uint32_t last_time = 0;
  static float R_last_err = 0.0;
  static float L_last_err = 0.0;
  static float R_err_integ = 0.0;
  static float L_err_integ = 0.0;  
  static const float d = 0.65; 

  static bool motor_power = false;
  float R_Vm, L_Vm;  // actual speeds calcurated by encorders

  static const bool debug = false;
  
  int8_t recvid;
  void* msgin;
  int ret = MP.Recv(&recvid, &msgin);
  if (ret > 0) {
    if (recvid == MOTOR_POWER_MSG) {
      std_msgs__msg__Bool* msg = (std_msgs__msg__Bool*)msgin;
      MPLog("motor_power: %d\n", msg->data);
      if (msg->data == true && motor_power == false) {
        digitalWrite(MOTOR_SW, HIGH);
        motor_power = true;
        // no need to set zero, but just in case
        VRt = VLt = 0.0; 
        R_err_integ = L_err_integ = 0.0;  
        MPLog("TurtleBotSPR ON\n");
      } else if (msg->data == false && motor_power == true) {
        digitalWrite(MOTOR_SW, LOW);
        motor_power = false;
        MPLog("TurtleBotSPR OFF\n");
        VRt = 0.0; VLt = 0.0;
        R_err_integ = L_err_integ = 0.0;  
      } 
    } else if (recvid == COMMAND_MSG) {
      if (motor_power == false) {
        MPLog("Motor Power is OFF\n");
        return;
      }
      geometry_msgs__msg__Twist* cmd = (geometry_msgs__msg__Twist*)msgin;
      MPLog("TurtleBotSPR set speed: %f\n", cmd->linear.x);
      MPLog("TurtleBotSPR set angular: %f\n", cmd->angular.z);
      VRt = cmd->linear.x + cmd->angular.z*d;
      VLt = cmd->linear.x - cmd->angular.z*d;
      MPLog("VR: %f, VL: %f\n", VRt, VLt);
      start_time = millis();
    } else if (recvid == REQ_ODOM) {
      int8_t sndid = REQ_ODOM;
      int ret = MP.Send(sndid, &rover_odm);
      if (ret < 0) { 
        printf("MP.Send odom error: %d\n", ret); 
      }
    }
  }

  if (motor_power == false) {
    VRt = VLt = 0.0;   
    R_Vm = L_Vm = 0.0;
    R_duty = L_duty = 0;
    R_last_err = L_last_err = 0.0;
    R_err_integ = L_err_integ = 0.0;
    delay(DELAY_TIME);
    return;
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

  noInterrupts();
  uint32_t cur_R = R; R = 0; 
  uint32_t cur_L = L; L = 0;
  interrupts();
  float R_mileage = 0.0;
  float L_mileage = 0.0;
  
  R_Vm = calc_speed(cur_R, duration, &R_mileage, VRt);
  L_Vm = calc_speed(cur_L, duration, &L_mileage, VLt);   
  if (debug) {
    MPLog("%d,%d,%d,%f,%f,%f,%f,%f,%f\n", 
      duration,cur_R,cur_L,R_Vm,L_Vm,R_mileage,L_mileage,VRt,VLt);
    MPLog("R_Vm: %f, L_Vm: %f, duration\n", R_Vm, L_Vm, duration);
  }

  // build up odometry topic
  if (abs(R_Vm) > 0.0 || abs(L_Vm) > 0.0) {
  
    // calc odometry
    update_odometry(R_Vm, L_Vn, duration_sec)

    rover_odm.odm_ang_z = odm_ang_z;
    rover_odm.odm_pos_x = odm_pos_x;
    rover_odm.odm_pos_y = odm_pos_y;
    rover_odm.odm_qt_qz = odm_qt_qz;
    rover_odm.odm_qt_qw = odm_qt_qw; 
  
    MPLog("%f, %f, %f, %f, %f\n", odm_ang_z, odm_pos_x, odm_pos_y, odm_qt_qz, odm_qt_qw);
  }  
   
  float duration_sec = duration/1000.0;
  R_err        = VRt - R_Vm;
  L_err        = VLt - L_Vm;
  R_err_integ += (R_err + R_last_err)*0.5*duration_sec;
  L_err_integ += (L_err + L_last_err)*0.5*duration_sec;
  R_derr       = (R_err - R_last_err)/duration_sec;  
  L_derr       = (L_err - L_last_err)/duration_sec;

  if (debug) {
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
