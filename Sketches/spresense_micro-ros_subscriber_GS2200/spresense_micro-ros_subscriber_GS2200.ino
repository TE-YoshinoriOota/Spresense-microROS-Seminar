#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_subscription_t subscriber;
static rclc_executor_t executor;
static rcl_timer_t timer;
std_msgs__msg__Int32 msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#include <micro_ros_arduino.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

#include <GS2200AtCmd.h>
#include <GS2200Hal.h>

#define ATCMD_CHECK(fn) { while(fn != ATCMD_RESP_OK); }

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

    // Initiallize SPI
    Init_GS2200_SPI();

    // check the io
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
    ATCMD_CHECK(AtCmd_WRXACTIVE(0));
    ATCMD_CHECK(AtCmd_WRXPS(1));
    ATCMD_CHECK(AtCmd_BDATA(1));

    // connect to your access point
    ATCMD_CHECK(AtCmd_WM(ATCMD_MODE_STATION));
    ATCMD_CHECK(AtCmd_WD());
    ATCMD_CHECK(AtCmd_NDHCP(1));
    ATCMD_CHECK(AtCmd_WPAPSK(ssid, pswd));
    ATCMD_CHECK(AtCmd_WA(ssid, "", 0));
    ATCMD_CHECK(AtCmd_WSTATUS());
    Serial.println("WiFi connected");

    while(true) {
      ATCMD_RESP_E r;
      r = AtCmd_NCUDP(server_ip, server_port, client_port, &client_id);
      if (r != ATCMD_RESP_OK) {
        ConsoleLog("No connect. Retry\n");
        delay(1000);
        continue;
      }
      
      if (client_id == ATCMD_INVALID_CID) {
        ConsoleLog("Got an invalid client id. Retry\n");
        delay(2000);
        continue;
      }
      ConsolePrintf("client_id: %d\r\n", client_id);
      
      ATCMD_CHECK(AtCmd_NSTAT(&net_status));
      ConsoleLog("Connected to the server\n");
      ConsolePrintf("IP: %d.%d.%d.%d\n"
        , net_status.addr.ipv4[0], net_status.addr.ipv4[1] 
        , net_status.addr.ipv4[2], net_status.addr.ipv4[3]);
      break;
         
    }
    return true;
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
      Serial.print(" ");
    }
    Serial.println();
#endif

    Serial.println("Start to send udp data: " + String(len) + " Byts");
    if (len == 0) return len;

    WiFi_InitESCBuffer();  
    // while (AtCmd_UDP_SendBulkData(client_id, buf, len, net_status.addr.ipv4, client_port) != ATCMD_RESP_OK) {
    if (AtCmd_SendBulkData(client_id, buf, len) != ATCMD_RESP_OK) {
      Serial.println("fails to send");
      return 0;
    }
    return len;
  }

  size_t arduino_wifi_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode) {
    (void) errcode;
    int res = 0;
    uint32_t start_time = millis();
    if (AtCmd_RecvResponse() == ATCMD_RESP_BULK_DATA_RX) {
      if (Check_CID(client_id)) {
        ConsolePrintf( "Receive %d bytes\r\n", ESCBufferCnt-1, ESCBuffer+1 );
        memcpy(buf, ESCBuffer+1, ESCBufferCnt-1);
        res = ESCBufferCnt-1;
      } else {
        Serial.println("clinet_id is not match: " + String(client_id));
      }
      WiFi_InitESCBuffer();
    }
#ifdef DEBUG
    Serial.println("arduino_wifi_transport_read: " + String(res));
    for (int n = 0; n < res; ++n) {
      Serial.print(buf[n], HEX);  
      Serial.print(" ");
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

void subscription_callback(const void * msgin) {  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED1, (msg->data == 0) ? LOW : HIGH);  
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
  RCCHECK(rclc_node_init_default(&node, "my_node_gs2200", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "my_subscriber_gs2200"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  digitalWrite(LED0, HIGH);  
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
