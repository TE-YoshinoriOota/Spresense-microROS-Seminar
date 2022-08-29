#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_init_options_t init_options;
static rcl_node_t node;
static rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#include <micro_ros_arduino.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

#include "ESP8266ATLib.h"
const char server_ip[16] = "192.168.xxx.xxx";
const char server_port[6] = "8888";

extern "C" {
  #define BAUDRATE 2000000

  bool arduino_wifi_transport_open(struct uxrCustomTransport* transport) {
    bool result = false;
    esp8266at.begin(BAUDRATE);
    result = esp8266at.espConnectAP("ssid","passwd");
    Serial.println("IP Address is " + esp8266at.getLocalIP());
    result = esp8266at.setupUdpClient(server_ip, server_port);
    return result;
  }

  bool arduino_wifi_transport_close(struct uxrCustomTransport* transport) {
    /* if this enables, the program will crash when the ros2 agent is not running on the network */
    /* Since micro-ROS calls the close-API and the write-API simultaneously,                     */
    /* the write-API may cause the probelm during closing of the network  (timing issue)         */
  }

  size_t arduino_wifi_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* errcode) {
    (void)errcode;
#ifdef DEBUG
    Serial.println("arduino_wifi_transport_write");
    for (int n = 0; n < len; ++n) {
      Serial.print(buf[n], HEX);
    }
    Serial.println();
#endif
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
    
#ifdef DEBUG
    Serial.println("arduino_wifi_transport_read: " + String(res));
    for (int n = 0; n < res; ++n) {
      Serial.print(buf[n], HEX);  
    }
    Serial.println();
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
  RCCHECK(rclc_node_init_default(&node, "my_node_esp8266", "", &support));
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "my_publisher_esp8266"));

  msg.data = 0;

  digitalWrite(LED0, HIGH);   
}


void loop() {
  delay(100);
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  msg.data++;
}
