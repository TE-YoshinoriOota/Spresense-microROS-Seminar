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

void error_loop(){
  while(1){
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

void setup() {
  set_microros_transports();

  delay(2000);
  
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   
  
  // create node 
  RCCHECK(rclc_node_init_default(&node, "my_node_serial", "", &support));
  
  // create publisher
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

  // Camera setup
  theCamera.begin();
  theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, CAM_IMAGE_PIX_FMT_JPG);
      
  digitalWrite(LED0, HIGH);   
}


void loop() {
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  CamImage img = theCamera.takePicture();
  if (img.isAvailable()) {
    if (img.getImgSize() > msg_static.data.capacity) {
      digitalWrite(LED3, HIGH);
    } else {
      msg_static.data.size = img.getImgSize();
      memcpy(msg_static.data.data, img.getImgBuff(), img.getImgSize());
      RCSOFTCHECK(rcl_publish(&publisher, &msg_static, NULL));
    }
  } else {
    digitalWrite(LED2, HIGH);
  }
}
