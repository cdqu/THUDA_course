#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import zmq
import sys
import time
import os
from random import randint
from rpi_ws281x import Adafruit_NeoPixel, Color
import smbus
import RPi.GPIO as GPIO
from apds9960.const import *
from apds9960 import APDS9960

num_agents = 3
#broker_ip = sys.argv[1]
#agent_id = int(sys.argv[2])
broker_ip = '192.168.137.230'
agent_id = 12

def intH(channel):
    print("INTERRUPT")

port = 1
bus = smbus.SMBus(port)
def scan_wifi():
    # 搜索附近的wifi
    os.system('wpa_cli -i wlan0 scan')
    # 显示搜索wifi热点的结果
    str = os.popen('wpa_cli -i wlan0 scan_result')
    return str

def main(agent_id):
    # 根据agent_id点亮不同位置的灯区分树莓派 1 2 3
    print('suceess0:the number of this pi is %d:',agent_id)
    strip = Adafruit_NeoPixel(32, 18, 800000, 10, False, 10)
    strip.begin()
    strip.setPixelColor(agent_id, Color(255, 0, 0))
    strip.show()
    
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.connect("tcp://%s:5556" % broker_ip)
    print("success1: begin zmq")
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://%s:5555" % broker_ip)
    # subscriber.setsockopt_string(zmq.SUBSCRIBE, str(agent_id))
    while True:
        flag = 0
        #message = subscriber.recv()
        r=scan_wifi()
        print("success2:scan_wifi")
        info = r.readlines()
        print("success3:read lines")
        for line in info:
            line = line.strip('\r\n')
            #print(line)
            mac = line[0:17]
            # 移动终端的mac地址
            if (mac == '54:25:ea:4c:22:c0') | (mac == '74:60:fa:81:a8:d8'):
                print('%d: %s' % (agent_id, line))
                publisher.send_string('%d: %s' % (agent_id, line))
                flag = 1
        if flag == 0:
            print('haha')
            time.sleep(0.2)
            
            
main(agent_id)
