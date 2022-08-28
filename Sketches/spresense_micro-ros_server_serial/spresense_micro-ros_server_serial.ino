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
  set_microros_transports();

  delay(2000);
  
  printf("*** rcl_get_default_allocator\n");
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   
  
  // create node 
  printf("*** rclc_node_init_default\n");
  RCCHECK(rclc_node_init_default(&node, "my_node_serial", "", &support));
  
  // create service
  printf("service_init_default\n");
  RCCHECK(rclc_service_init_default(&service, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "srv_trigger_serial"));
 
  printf("executor_init\n");  
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));  

  digitalWrite(LED0, HIGH);   
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  
}
