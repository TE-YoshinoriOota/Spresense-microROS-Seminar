#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/empty.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <nav_msgs/msg/odometry.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

static rcl_subscription_t   msw_subscriber;  // motor switch subscriber
static rcl_subscription_t   res_subscriber;  // reset subscriber
static rcl_subscription_t   pic_subscriber;  // camera shutter subscriber
static rcl_subscription_t   cmd_subscriber;  // command subscriber
static rcl_publisher_t      img_publisher;   // camera image publisher
static rcl_publisher_t      odm_publisher;   // odometry publisher
std_msgs__msg__Bool  msw;             // motor switch message
std_msgs__msg__Empty res;             // reset message
std_msgs__msg__Empty pic;             // take picture message
geometry_msgs__msg__Twist cmd_vel;    // command message
sensor_msgs__msg__CompressedImage msg_static;  // camera image entity
nav_msgs__msg__Odometry odm;          // odometry message

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer;

// health check function for 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#include <MP.h>
#define MOTOR_POWER_MSG 100
#define COMMAND_MSG     101
#define REQ_ODOM        102
const int subcore = 1;

struct rover_odm {
  float odm_ang_z;
  float odm_pos_x;
  float odm_pos_y;
  float odm_qt_qz;
  float odm_qt_qw;
};

#include <Camera.h>
const int img_buffer_size = 25000;

#include <LowPower.h>

#include <micro_ros_arduino.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

#define ATCMD_CHECK(fn) { while(fn != ATCMD_RESP_OK); }

#include <GS2200AtCmd.h>
#include <GS2200Hal.h>

ATCMD_NetworkStatus net_status;
extern uint8_t ESCBuffer[];
extern uint32_t ESCBufferCnt;
const char server_ip[16] = "192.168.2.101";
const char server_port[6] = "8888";
const char client_port[6] = "10001";
uint8_t client_id = 0;

extern "C" {
  bool arduino_wifi_transport_open(struct uxrCustomTransport* transport) {
    ATCMD_REGDOMAIN_E regDomain;
    const char ssid[32] = "elecom2g-495968";
    const char pswd[32] = "1360758451781";
    char macid[20];

    // Initialize SPI
    Init_GS2200_SPI();
    
    // wait for the ready
    while (Get_GPIO37Status()) {
      if (AtCmd_RecvResponse() == ATCMD_RESP_NORMAL_BOOT_MSG) 
        ConsoleLog("Normal Boot\r\n");
    }

    // setup
    ATCMD_CHECK(AtCmd_AT());
    ATCMD_CHECK(AtCmd_ATE(0));
    ATCMD_CHECK(AtCmd_WREGDOMAIN_Q(&regDomain));
    if (regDomain != ATCMD_REGDOMAIN_TELEC) {
      ATCMD_CHECK(AtCmd_WREGDOMAIN(ATCMD_REGDOMAIN_TELEC));
    }
    ATCMD_CHECK(AtCmd_NMAC_Q(macid));
    ATCMD_CHECK(AtCmd_VER());
    ATCMD_CHECK(AtCmd_WRXACTIVE(1)); // non power save mode
    ATCMD_CHECK(AtCmd_WRXPS(0)); // non power save mode
    ATCMD_CHECK(AtCmd_BDATA(1));

    // connect to your access point
    ATCMD_CHECK(AtCmd_WM(ATCMD_MODE_STATION));
    ATCMD_CHECK(AtCmd_WD());
    ATCMD_CHECK(AtCmd_NDHCP(1));
    ATCMD_CHECK(AtCmd_WPAPSK(ssid, pswd));
    ATCMD_CHECK(AtCmd_WA(ssid, "", 0));
    ATCMD_CHECK(AtCmd_WSTATUS());
    MPLog("WiFi connected");

    struct micro_ros_agent_locator* locator = (struct micro_ros_agent_locator*)transport->args;
    while(true) {
      ATCMD_RESP_E r;
      r = AtCmd_NCUDP(server_ip, server_port, client_port, &client_id);
      if (r != ATCMD_RESP_OK) {
        MPLog("No connect. Retry\n");
        delay(1000);
        continue;
      }
      
      if (client_id == ATCMD_INVALID_CID) {
        MPLog("Got an invalid client id. Retry\n");
        delay(2000);
        continue;
      }
      MPLog("client_id: %d\r\n", client_id);
      
      ATCMD_CHECK(AtCmd_NSTAT(&net_status));
      MPLog("Connected to the server\n");
      MPLog("IP: %d.%d.%d.%d\n"
        , net_status.addr.ipv4[0], net_status.addr.ipv4[1] 
        , net_status.addr.ipv4[2], net_status.addr.ipv4[3]);
      break;
    }
    digitalWrite(LED0, HIGH); 
    return true;
  }

  bool arduino_wifi_transport_close(struct uxrCustomTransport* transport) {
    /* if this enables, the program will crash when the ros2 agent is not running on the network */
    /* Since micro-ROS calls the close-API and the write-API simultaneously,                     */
    /* the write-API may cause the probelm during closing of the network  (timing issue)         */
    return true;
  }

  size_t arduino_wifi_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* errcode) {
    (void)errcode;
#ifdef DEBUG
    MPLog("arduino_wifi_transport_write\n");
    for (int n = 0; n < len; ++n) {
      MPLog("0x%02X ", buf[n]);
    }
    MPLog("\n");
#endif
    MPLog("> Send udp data: %d Bytes\n", len);
    WiFi_InitESCBuffer(); 
    if (len != 0) {
      int retry = 0; 
      while (AtCmd_SendBulkData(client_id, buf, len) != ATCMD_RESP_OK) {
        MPLog("fails to send. retry.\n");
        delay(10);
        if (retry++ > 10) {
          MPLog("give up to send\n");
          len = 0;
        }
      }
    }
    return len;
  }

  size_t arduino_wifi_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode) {
    (void) errcode;
    int res = 0;
    if (AtCmd_RecvResponse() == ATCMD_RESP_BULK_DATA_RX) {
      if (Check_CID(client_id)) {
        MPLog( "< Receive udp data: %d bytes\n", ESCBufferCnt-1);
        memcpy(buf+res, ESCBuffer+1, ESCBufferCnt-1);
        res = ESCBufferCnt-1;
      } else {
        MPLog("clinet_id is not match: %d\n", client_id);
      }
      WiFi_InitESCBuffer();
    }
#ifdef DEBUG
    MPLog("arduino_wifi_transport_read: %d\n", res);
    for (int n = 0; n < res; ++n) {
      MPLog("0x%02X ", buf[n]);
    }
    MPLog("\n");
#endif
    return res;
  }
}

void error_loop(){
  while(1){
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

void msw_callback(const void * msgin) { 
  std_msgs__msg__Bool* msw_ = (std_msgs__msg__Bool*)msgin;
  MPLog("msg_callback: %d\n", msw_->data); 
  int8_t sndid = 100;
  static std_msgs__msg__Bool msgout;
  memcpy(&msgout, msw_, sizeof(std_msgs__msg__Bool));
  int ret = MP.Send(sndid, &msgout, subcore);
  if (ret < 0) {
    MPLog("MP.Send error = %d\n", ret);
  }
  digitalWrite(LED1, (msw_->data == 0) ? LOW : HIGH);  
}

void cmd_vel_callback(const void * msgin) {
  geometry_msgs__msg__Twist* cmd_ = (geometry_msgs__msg__Twist*)msgin;
  MPLog("cmd_vel_callback: %f\n", cmd_->linear.x); 
  int8_t sndid = 101;
  static geometry_msgs__msg__Twist cmdout;
  memcpy(&cmdout, cmd_, sizeof(geometry_msgs__msg__Twist));
  // send the command to subcore  
  int ret = MP.Send(sndid, &cmdout, subcore);
  if (ret < 0) {
    MPLog("MP.Send error = %d\n", ret);
  }  
}

void pic_callback(const void * msgin) { 
  UNUSED(msgin);
  digitalWrite(LED2, HIGH);
  MPLog("take picture\n");
  CamImage img = theCamera.takePicture();
  if (img.isAvailable()) {
    if (img.getImgSize() > msg_static.data.capacity) {
      MPLog("taken image size is over msg size\n");
      MPLog("taken image size : %d\n", img.getImgSize());
      MPLog("msg.data.capacity: %d\n", msg_static.data.capacity);
    } else {
      MPLog("taken image size is %d\n", img.getImgSize());
      msg_static.data.size = img.getImgSize();
      memset(msg_static.data.data, NULL, img_buffer_size*sizeof(uint8_t));
      memcpy(msg_static.data.data, img.getImgBuff(), img.getImgSize());
      
      // publish image
      MPLog("rcl_publish\n");
      RCCHECK(rcl_publish(&img_publisher, &msg_static, NULL));
    }
  } else {
    MPLog("take image failure..\n");
  }
  digitalWrite(LED2, LOW);
}

void res_callback(const void * msgin) { 
  UNUSED(msgin);
  MPLog("system reset\n");
  LowPower.reboot();
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  int8_t sndid=REQ_ODOM;
  int snd_empty = 0;
  int8_t recvid;
  struct rover_odm* rover_odm;
  if (timer != NULL) {
    printf("publish odometry \n");
    MP.Send(sndid, snd_empty, subcore);
    int ret = MP.Recv(&recvid, &rover_odm, subcore);
    if (ret >= 0) {
      odm.twist.twist.angular.z = rover_odm->odm_ang_z;
      odm.pose.pose.position.x = rover_odm->odm_pos_x;
      odm.pose.pose.position.y = rover_odm->odm_pos_y;
      odm.pose.pose.orientation.z = rover_odm->odm_qt_qz;
      odm.pose.pose.orientation.w = rover_odm->odm_qt_qw;
      MPLog("ODOM: %f,%f,%f,%f,%f\n", 
        rover_odm->odm_ang_z, 
        rover_odm->odm_pos_x, rover_odm->odm_pos_y,
        rover_odm->odm_qt_qz, rover_odm->odm_qt_qw);
    }

    // get odometry data from subcore-1
    uint32_t current_time_in_micros = micros();
    odm.header.stamp.sec = current_time_in_micros/1000000;
    odm.header.stamp.nanosec = (current_time_in_micros - (odm.header.stamp.sec)*1000000)*1000; 
    // publish odometry
    RCSOFTCHECK(rcl_publish(&odm_publisher, &odm, NULL));
  }
}


void setup() {
  Serial.begin(115200);
  rmw_uros_set_custom_transport(
    false, NULL,
    arduino_wifi_transport_open,
    arduino_wifi_transport_close,
    arduino_wifi_transport_write,
    arduino_wifi_transport_read
  );
  
  delay(2000);
  
  allocator = rcl_get_default_allocator();

  MPLog("Initialize micro-ROS\n");

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   

  MPLog("create micro-ROS node  <turtlebot_spr_node>\n");
  RCCHECK(rclc_node_init_default(&node, "turtlebot_spr_node", "", &support));

  MPLog("Create subscriber for topic <motor_power> \n");
  RCCHECK(rclc_subscription_init_default(
    &msw_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor_power"));

  MPLog("Create subscriber for topic <cmd_vel> \n");
  RCCHECK(rclc_subscription_init_default(
    &cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  MPLog("Create subscriber for topic <take_picture> \n");
  RCCHECK(rclc_subscription_init_default(
    &pic_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "take_picture"));

  MPLog("Create subscriber for topic <reset> \n");
  RCCHECK(rclc_subscription_init_default(
    &res_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "reset"));

  MPLog("create publisher for topic <image/compressed> \n");
  RCCHECK(rclc_publisher_init_default(
    &img_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    "image/compressed"));

  MPLog("create publisher for odometry <odom> \n");
  RCCHECK(rclc_publisher_init_default(
    &odm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // create buffer for img_publisher
  static uint8_t img_buff[img_buffer_size] = {0};
  static char frame_id_data[30] = {0};
  static char format_data[6] = {0};
  msg_static.header.frame_id.capacity = 7;
  msg_static.header.frame_id.data=frame_id_data;
  sprintf(msg_static.header.frame_id.data, "myframe");
  msg_static.header.frame_id.size = 7;
  msg_static.data.capacity = img_buffer_size;
  msg_static.data.data = img_buff;
  msg_static.data.size = 0;
  msg_static.format.capacity = 4;
  msg_static.format.data = format_data;
  sprintf(msg_static.format.data, "jpeg");  
  msg_static.format.size = 4;

  // crate Odemetry msgs
  static char odom[5] = {0};
  odm.header.frame_id.data = odom;
  sprintf(odm.header.frame_id.data, "odom");
  odm.header.frame_id.size = 4;
  odm.header.frame_id.capacity=5;

  // create timer,
  MPLog("create timer \n");
  const uint32_t timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  MPLog("initialize rclc_executor\n");  
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &msw_subscriber, &msw, &msw_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pic_subscriber, &pic, &pic_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &res_subscriber, &res, &res_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  MPLog("Camera begin\n");
  theCamera.begin();
  theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, CAM_IMAGE_PIX_FMT_JPG);  

  MPLog("Subcore start\n");
  MP.begin(subcore);
  digitalWrite(LED0, HIGH);  
}

void loop() {
  delay(100);
  // periodic execution
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
