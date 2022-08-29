#ifndef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <MPMutex.h>
MPMutex mutex(MP_MUTEX_ID0);


#include <GS2200AtCmd.h>
#include <GS2200Hal.h>

#define  AP_SSID        "ssid"
#define  PASSPHRASE     "passwd"

#define  TCPSRVR_IP     "192.168.xxx.xxx"
#define  TCPSRVR_PORT   "8080"

extern uint8_t ESCBuffer[];
extern uint32_t ESCBufferCnt;
char server_cid = 0;
bool b_connected = false;

const int recv_buf_size = 32;
char recv_buf[recv_buf_size] = {0};

#define DELAY_TIME (40)

const int param_num = 8;
float param[param_num];

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

struct pid_data pid;

void get_param_task() {
  while (true) {
    if (update_param(recv_buf, recv_buf_size)) {
      if (parse_param(recv_buf, param)) {
        pid.rKp = param[0];
        pid.rKi = param[1];
        pid.rKd = param[2];
        pid.lKp = param[3];
        pid.lKi = param[4];
        pid.lKd = param[5];
        pid.vt  = param[6];
        pid.rot = param[7];
        int8_t snd_id = 100;
        MP.Send(snd_id, &pid);    
      }
    }
    sleep(1);
  }
  
}

bool parse_param(char* recv_buf, float* param) {
  String str(recv_buf);
  str.trim();
  MPLog("params: \n");
  for (int n = 0; n < param_num-1; ++n) {
    int index = str.indexOf(',');
    if (index < 0) {
      MPLog("Cannot find commna\n");
      return false;
    }
    String left_str = str.substring(0, index);
    String right_str = str.substring(index+1, str.length());
    // MPLog("%s|$s\n", left_str.c_str(), right_str.c_str());
    param[n] = atof(left_str.c_str());
    MPLog("%f,", param[n]);
    str = right_str;
  }
  param[param_num-1] = atof(str.c_str());
  MPLog("\n");
  return true;
}

void setup() {
  wifi_begin();
  wifi_connectAP();
  connect_server();
  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING); 
  task_create("get_param", 100, 1024, get_param_task, NULL); 
}

void loop() {
  static uint32_t last_R_duty = 0;
  static uint32_t last_L_duty = 0;
  static String pid_log = "";
  int8_t rid;
  struct torque_data* td;
  int ret = MP.Recv(&rid, &td);
  if (ret > 0) {
    if (last_R_duty != td->R_duty || last_L_duty != td->L_duty) {
      pid_log = String(td->R_duty) + "," + String(td->L_duty) + "\n";
      send_data(pid_log);
      last_R_duty = td->R_duty;
      last_L_duty = td->L_duty;
    }
  }
  usleep(DELAY_TIME*1000);
}
