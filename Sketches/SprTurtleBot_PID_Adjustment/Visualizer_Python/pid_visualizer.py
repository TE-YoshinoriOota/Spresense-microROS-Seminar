# -*- coding : UTF-8 -*-
import socket
import asyncio
import aioconsole
import time
import numpy as np
import threading
import matplotlib.pyplot as plt
import collections

server_ip = "192.168.2.101"
server_port = 8080
listen_num = 5
buffer_size = 1024

tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tcp_server.bind((server_ip, server_port))
tcp_server.listen(listen_num)

bRunning = True
history = collections.deque(maxlen=100)

def recv_run():
  try:
    while True:
      global bRunning
      if (bRunning == False):
        break
      data = client.recv(buffer_size)
      data = data.decode()
      l = [int(y.strip()) for y in data.split(',')]
      history.append(l)
      print("[*] Formated Data : {}".format(l))
      time.sleep(0.01)  
    client.close()
    print("connection closed")
    return
  except:
    client.close()
    print("connection closed by exception")
    bRunning = False

def cmd_input():
  while True:
    line = input("input: ")
    if line == str('q'):
      print("program will terminate soon")
      print("please wait for connection timeout...")
      print("OR please type ^c if this program does not terminate...")
      global bRunning
      bRunning = False
      stop_line = str("0,0,0,0,0,0,0,0")
      client.send(bytes(stop_line, 'utf-8'))    
      return 
    client.send(bytes(line, 'utf-8'))    
    print("[*] Send Data : {}".format(line))

if __name__=="__main__":
  print("Waiting for SprTurtleBot access");
  print("Please reset SprTurtleBot and wait for a moment...");
  client, address = tcp_server.accept()
  print("Connected!! [ SprTrutleBot IP : {}]".format(address))

  t1 = threading.Thread(target=cmd_input)
  t1.start()

  plt.ion()
  fig,ax = plt.subplots()
  #client.settimeout(10)

  t2 = threading.Thread(target=recv_run)
  t2.start()

  while True:
    if (bRunning == False):
      break
    try: 
      for i in range(30): # frame 
        x = list(range(i-len(history), i))
        plt.plot(x,history) 
        plt.xlabel("frame")
        plt.ylabel("torque")
        plt.draw() 
        plt.pause(0.1) 
        plt.cla()
    except KeyboardInterrupt:
      plt.close()

