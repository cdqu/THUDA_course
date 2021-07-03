#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import zmq
import sys
import time
import os
from random import randint
import smbus
import threading
from rpi_ws281x import Adafruit_NeoPixel, Color

num_agents = 3
# broker_ip = sys.argv[1]
# agent_id = int(sys.argv[2])
# 主节点的ip地址
broker_ip = '192.168.137.175'
agent_id = 1
send_flag = False

# 设置全局变量保存移动终端的数据
global info1 = ''
global info2 = ''
global info3 = ''

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

# 接收移动终端的热点
def recv_wifi(threadName, delay):
    print("success3:recv from phone")
    while True:
        r=scan_wifi()
        info = r.readlines()
        for line in info:
            #print(line)
            line = line.strip('\r\n')
            mac = line[0:17]

            # 移动终端的mac地址
            if (mac == '54:25:ea:4c:22:c0') | (mac == '74:60:fa:81:a8:d8'):
                # 输出line
                print(line) # 看看格式
                print('recv_wifi %d: %s' % (agent_id, line))
                # 输出mac rssi name
                rssi = line[22:24]
                line_length = length(line)
                name = [45:line_length]
                print('mac: %s, rssi:%s, name:%s ',mac, rssi, name)
#                 # 是否需要发送给其他树莓派
#                 if send_flag:
#                     publisher.send_string('%d: %d' % (randint(1, num_agents), line))

        time.sleep(delay)
        
# 扫描信道接收来自树莓派的
def scanner(threadName, delay):
    print("success4:begin scanner for pi")
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.connect("tcp://%s:5556" % broker_ip)

    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://%s:5555" % broker_ip)
    # 筛选开头为'1'的信息
    subscriber.setsockopt_string(zmq.SUBSCRIBE, str(1))
    while True:
        print('s5')
        message = subscriber.recv()
        print('s6:recv from pi')
        mac = line[0:17]
        rssi = line[22:24]
        line_length = length(line)
        name = [45:line_length]
        print(message)
        time.sleep(delay)
        
try:
    threading.start_new_thread( print_time, ("Thread-1", 0.2,))
    threading.start_new_thread( print_time, ("Thread-2", 0.2,))
except:
    print ("Error: unable to start thread")

exitFlag = 0
global val
# 定义线程
# 继承父类threading.Thread
class myThread1 (threading.Thread):   
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        # 接收移动终端的热点
        if self.threadID == 1:           
            recv_wifi(self.name, 0.2)
        # 发送
        elif self.threadID == 2:
            scanner(self.name, 0.2)


def main():
    # 根据agent_id点亮不同位置的灯区分树莓派 1 2 3
    print('suceess0:the number of this pi is %d:',agent_id)
    strip = Adafruit_NeoPixel(32, 18, 800000, 10, False, 10)
    strip.begin()
    strip.setPixelColor(agent_id, Color(255, 0, 0))
    strip.show()
    
#     context = zmq.Context()
#     publisher = context.socket(zmq.PUB)
#     publisher.connect("tcp://%s:5556" % broker_ip)

#     subscriber = context.socket(zmq.SUB)
#     subscriber.connect("tcp://%s:5555" % broker_ip)
#     subscriber.setsockopt_string(zmq.SUBSCRIBE, str(agent_id))
    print('suceess1:begin thread')
    # 创建新线程
    thread1 = myThread1(1, "Thread-1", 1)
    thread2 = myThread1(2, "Thread-2", 2)
 
    # 开启线程
    thread1.start()
    thread2.start()
    
#     while True:
#         message = subscriber.recv()
#         lineothers = int(message[2:])
#         r=scan_wifi()
#         info = r.readlines()
#         for line in info:
#             line = line.strip('\r\n')
#             mac = line[0:17]
#             if mac == '74:60:fa:81:a8:d8':
#                 print('%d: %s' % (agent_id, line))
#                 #publisher.send_string('%d: %d' % (randint(1, num_agents), line))

#             time.sleep(0.2)


        #publisher.send_string('%d: %s' % (randint(1, num_agents), 'activate'))


# In[ ]:


main()

