
void wifi_begin(void) {
  ATCMD_RESP_E r = ATCMD_RESP_UNMATCH;
  ATCMD_REGDOMAIN_E regDomain;
  char macid[20];

  Init_GS2200_SPI();
  AtCmd_Init();

  /* Try to read boot-up banner */
  while (Get_GPIO37Status()){
    if (AtCmd_RecvResponse() == ATCMD_RESP_NORMAL_BOOT_MSG) {
      Serial.println("Normal Boot");      
    }
  } 
  while (ATCMD_RESP_OK != AtCmd_AT());
  /* Send command to disable Echo */
  while (ATCMD_RESP_OK != AtCmd_ATE(0));

  /* AT+WREGDOMAIN=? should run after disabling Echo, otherwise the wrong domain is obtained. */
  while (ATCMD_RESP_OK != AtCmd_WREGDOMAIN_Q(&regDomain));
  if (regDomain != ATCMD_REGDOMAIN_TELEC) {
    while (ATCMD_RESP_OK != AtCmd_WREGDOMAIN(ATCMD_REGDOMAIN_TELEC));
  } 
  /* Read MAC Address */
  while (ATCMD_RESP_OK != AtCmd_NMAC_Q(macid)); 
  /* Read Version Information */
  while (ATCMD_RESP_OK != AtCmd_VER());
  /* Disable Power save mode */
  while (ATCMD_RESP_OK != AtCmd_WRXACTIVE(1)); 
  while (ATCMD_RESP_OK != AtCmd_WRXPS(1)); 
  /* Bulk Data mode */
  while (ATCMD_RESP_OK != AtCmd_BDATA(1)); 
}


void wifi_connectAP(void) {
  ATCMD_RESP_E r;
#ifdef APP_DEBUG
  ConsolePrintf("Associating to AP: %s\r\n", AP_SSID);
#endif
  /* Set Infrastructure mode */
  while (ATCMD_RESP_OK != AtCmd_WM(ATCMD_MODE_STATION));
  /* Try to disassociate if not already associated */
  while (ATCMD_RESP_OK != AtCmd_WD());
  /* Enable DHCP Client */
  while (ATCMD_RESP_OK != AtCmd_NDHCP(1)); 
  /* Set WPA2 Passphrase */
  while (ATCMD_RESP_OK != AtCmd_WPAPSK((char*)AP_SSID, (char*)PASSPHRASE)); 
  /* Associate with AP */
  while (ATCMD_RESP_OK != AtCmd_WA((char*)AP_SSID, (char*)"", 0)); 
  /* L2 Network Status */
  while (ATCMD_RESP_OK != AtCmd_WSTATUS()); 
}

void connect_server(void) {
  ATCMD_RESP_E resp = ATCMD_RESP_UNMATCH;
  ATCMD_NetworkStatus networkStatus;
  // Start a TCP client
  ConsoleLog( "Start TCP Client");
  resp = AtCmd_NCTCP((char*)TCPSRVR_IP, (char*)TCPSRVR_PORT, &server_cid);
  if (resp != ATCMD_RESP_OK) {
    MPLog("No Connect!\n");
    delay(2000); return;
  }
  if (server_cid == ATCMD_INVALID_CID) {
    MPLog("No CID!\n");
    delay(2000); return;
  }
  
  while (ATCMD_RESP_OK != AtCmd_NSTAT(&networkStatus));
  
  MPLog("Connected\n");
  MPLog("IP: %d.%d.%d.%d\r\n", networkStatus.addr.ipv4[0], networkStatus.addr.ipv4[1], networkStatus.addr.ipv4[2], networkStatus.addr.ipv4[3]);
  b_connected = true;
}

bool update_param(char* recv_buf, int recv_buf_size) {
  ATCMD_RESP_E resp = ATCMD_RESP_UNMATCH; 
  if (Get_GPIO37Status()) return false;  
  if (ATCMD_RESP_BULK_DATA_RX == AtCmd_RecvResponse()) {
    memset(recv_buf, NULL, recv_buf_size);
    if (Check_CID(server_cid)) {
      if (ESCBufferCnt-1 > recv_buf_size) {
        MPLog("Recv Buffer overflow\n");
      } else {
        memcpy(recv_buf, ESCBuffer+1, ESCBufferCnt-1);
        MPLog("Receive data(orig) is %s\n", ESCBuffer+1);        
        MPLog("Receive data(copy) is %s\n", recv_buf);
        WiFi_InitESCBuffer();
        return true;
      }     
    }
    WiFi_InitESCBuffer();
  }
  return false;
}

void send_data(String str) {
  WiFi_InitESCBuffer();  
  MPLog("send data %s", str.c_str());
  if (ATCMD_RESP_OK != AtCmd_SendBulkData(server_cid, str.c_str(), str.length())){
    MPLog("Send Error: %s\n", str.c_str());
  }
}
