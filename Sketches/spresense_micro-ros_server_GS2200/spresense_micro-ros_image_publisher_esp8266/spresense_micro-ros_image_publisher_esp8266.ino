#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/compressed_image.h>

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static sensor_msgs__msg__CompressedImage msg_static;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ printf("soft error\n");}}

#include <Camera.h>

#include <micro_ros_arduino.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

#include "ESP8266ATLib.h"
const char server_ip[16] = "192.168.2.101";
const char server_port[6] = "8888";

extern "C" {
  #define BAUDRATE 2000000

  bool arduino_wifi_transport_open(struct uxrCustomTransport* transport) {
    bool result = false;
    esp8266at.begin(BAUDRATE);
    result = esp8266at.espConnectAP("elecom2g-495968","1360758451781");
    Serial.println("IP Address is " + esp8266at.getLocalIP());
    result = esp8266at.setupUdpClient(server_ip, server_port);
    return result;
  }
  
  bool arduino_wifi_transport_close(struct uxrCustomTransport* transport) {
    /* if this enables, the program will crash when the ros2 agent is not running on the network */
    /* Since micro-ROS calls the close-API and the write-API simultaneously,                     */
    /* the write-API may cause the probelm during closing of the network  (timing issue)         */
    return true;
  }
  
  size_t arduino_wifi_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* errcode) {
    (void)errcode;
    bool result = esp8266at.sendUdpMessageToServer(buf, len);
    if (result) return len;
    else return 0;
  }
  
  size_t arduino_wifi_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode) {
    (void) errcode;
    int res = 0;
    uint32_t start_time = millis();
    do {
      res = esp8266at.espListenToUdpServer(buf, len);
    } while(!res && (millis() - start_time < timeout));
    return res;
  }
}

void error_loop(){
  while(1){
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
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

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   
  
  // create node 
  printf("*** rclc_node_init_default\n");
  RCCHECK(rclc_node_init_default(&node, "my_node_esp8266", "", &support));
  
  // create publisher
  printf("*** rclc_publisher_init_best_effort\n");
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    "image/compressed"));

  static uint8_t img_buff[20000] = {0};
  static char    frame_id_data[30] = {0};
  static char    format_data[6] = {0};
  msg_static.header.frame_id.capacity = 7;
  msg_static.header.frame_id.data=frame_id_data;
  memcpy(msg_static.header.frame_id.data, "myframe", 7);
  msg_static.header.frame_id.size = 7;
  msg_static.data.capacity = 20000;
  msg_static.data.data=img_buff;
  msg_static.data.size = 0;
  msg_static.format.capacity = 4;
  msg_static.format.data=format_data;
  memcpy(msg_static.format.data, "jpeg", 4);
  msg_static.format.size = 4; 
  printf("*** created message_memory\n");  

  // Camera setup
  theCamera.begin();
  theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, CAM_IMAGE_PIX_FMT_JPG);

  digitalWrite(LED0, HIGH);   
}

void loop() {
  printf("take picture\n");
  CamImage img = theCamera.takePicture();
  if (img.isAvailable()) {
    if (img.getImgSize() > msg_static.data.capacity) {
      Serial.println("taken image size is over msg size");
      Serial.println("taken image size : " + String(img.getImgSize()));
      Serial.println("msg.data.capacity: " + String(msg_static.data.capacity));
    } else {
      Serial.println("taken image size is " + String(img.getImgSize()));
      msg_static.data.size = img.getImgSize();
      memcpy(msg_static.data.data, img.getImgBuff(), img.getImgSize());
      RCSOFTCHECK(rcl_publish(&publisher, &msg_static, NULL));
    }
  } else {
    Serial.println("take image failure..");
  }
}
