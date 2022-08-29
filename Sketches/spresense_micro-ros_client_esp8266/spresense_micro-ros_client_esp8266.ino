#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/trigger.h>

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_client_t client;
static rclc_executor_t executor;
std_srvs__srv__Trigger_Request req;

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
    return true;
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

static void client_callback(const void * msg){
  digitalWrite(LED1, !digitalRead(LED1));
  std_srvs__srv__Trigger_Response* msg_in = (std_srvs__srv__Trigger_Response*)msg;
  printf("Received service response  %d\n", msg_in->success);
  char* ptr = msg_in->message.data;
  ++ptr; // avoid the bug of miro-ros-agent setting a null of the head of string
  printf("Received service message  %s, %d\n", ptr, msg_in->message.size);
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
  
  // create client
  RCCHECK(rclc_client_init_default(&client, &node, 
     ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "srv_trigger_py"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  std_srvs__srv__Trigger_Response res; 
  RCCHECK(rclc_executor_add_client(&executor, &client, &res, client_callback));    

  digitalWrite(LED0, HIGH);   
}

void loop() {
  int64_t seq;
  sleep(2); // Sleep a while to ensure DDS matching before sending request
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  printf("Send Trigger to server.\n"); 
  RCCHECK(rcl_send_request(&client, &req, &seq));
}
