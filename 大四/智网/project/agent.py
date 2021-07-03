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
broker_ip = '192.168.137.163'
agent_id = 1

def intH(channel):
    print("INTERRUPT")

port = 1
bus = smbus.SMBus(port)


# LED_COLOR      = (255, 0, 0) # color of active LED
# LED_COUNT      = 32      # Number of LED pixels.
# LED_PIN        = 18      # GPIO pin connected to the pixels (must support PWM!).
# LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
# LED_DMA        = 10       # DMA channel to use for generating signal (try 5)
# LED_BRIGHTNESS = 10    # Set to 0 for darkest and 255 for brightest
# LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)

flag = 1
score = 0
#strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
#strip.begin()

def scan_wifi():
    # 搜索附近的wifi
    os.system('wpa_cli -i wlan0 scan')
    # 显示搜索wifi热点的结果
    str = os.popen('wpa_cli -i wlan0 scan_result')
    return str

def main(agent_id):
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.connect("tcp://%s:5556" % broker_ip)
    print("success1")
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://%s:5555" % broker_ip)
    #subscriber.setsockopt_string(zmq.SUBSCRIBE, str(agent_id))
    while True:
        #message = subscriber.recv()
        r=scan_wifi()
        print("success2")
        info = r.readlines()
        print("success3")
        for line in info:
            line = line.strip('\r\n')
            #print(line)
            mac = line[0:17]
            if mac == '54:25:ea:4c:22:c0':
                print('%d: %s' % (agent_id, line))
                publisher.send_string('%d: %s' % (1, line))
                
#         print(line)
#         score = int(message[2:])
#         print('%d: %s' % (agent_id, message))
        # YOUR CODE HERE: add code for the sensors and LED HAT

        
            publisher.send_string('%d: %d' % (randint(1, num_agents), score))

                

            time.sleep(0.2)


        #publisher.send_string('%d: %s' % (randint(1, num_agents), 'activate'))

main(agent_id)
