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
static rcl_service_t service;
static rclc_executor_t executor;

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

std_srvs__srv__Trigger_Response res;
std_srvs__srv__Trigger_Request req; 
const int capacity = 32;
uint8_t data[capacity] = {0};

void service_callback(const void * req, void * res){
  static bool result = false;
  static int cnt = 0;
  std_srvs__srv__Trigger_Request * req_in = (std_srvs__srv__Trigger_Request *) req;
  std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;
  sprintf(data, "Response[%d]", cnt);
  res_in->success = !result; result = res_in->success;
  res_in->message.capacity=capacity;
  res_in->message.size = strlen(data);
  res_in->message.data = data;
  printf("Send Response: %d %s\n", res_in->success, res_in->message.data); 
  printf("Message detail: %s %d %d\n", res_in->message.data, res_in->message.size, res_in->message.capacity);
  ++cnt;
  digitalWrite(LED1, result);
}


void setup() {

  Serial.begin(115200);

  rmw_uros_set_custom_transport(
    false,
    NULL,
    arduino_wifi_transport_open,
    arduino_wifi_transport_close,
    arduino_wifi_transport_write,
    arduino_wifi_transport_read
  );
  
  delay(2000);

  printf("*** rcl_get_default_allocator\n");
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   
  
  // create node 
  printf("*** rclc_node_init_default\n");
  RCCHECK(rclc_node_init_default(&node, "my_node_esp8266", "", &support));
  
  // create service
  printf("service_init_default\n");
  RCCHECK(rclc_service_init_default(&service, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "srv_trigger_esp8266"));
 
  printf("executor_init\n");  
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));  

  digitalWrite(LED0, HIGH);   
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  
}
