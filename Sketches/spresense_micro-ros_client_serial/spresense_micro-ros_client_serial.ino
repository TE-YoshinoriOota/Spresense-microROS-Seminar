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
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   
  
  // create node 
  RCCHECK(rclc_node_init_default(&node, "my_node_serial", "", &support));
  
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
