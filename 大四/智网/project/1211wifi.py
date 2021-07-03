# -*- coding:utf-8 -*-
import os

def set_wifi_ssid_psk(ssid, psk):
    os.system('sudo wpa_cli -i wlan0 set_network 0 ssid ' + '\'"' + ssid + '"\'')
    os.system('sudo wpa_cli -i wlan0 set_network 0 psk ' + '\'"' + psk + '"\'')
    os.system('sudo wpa_cli -i wlan0 enable_network 0')
    os.system('sudo wpa_cli -i wlan0 save_config')

def scan_wifi():
    # 搜索附近的wifi
    os.system('wpa_cli -i wlan0 scan')
    # 显示搜索wifi热点的结果
    str = os.popen('wpa_cli -i wlan0 scan_result')
    return str
    
# def main():
#     # set_wifi_ssid_psk('ssid_set_by_python', 'psk_set_by_python')


# if __name__ == '__main__':
#     main()



r=scan_wifi()
info = r.readlines()
tmp = 0
for line in info:
    line = line.strip('\r\n')
    mac = line[0:17]
    print(line)
    if mac == '74:60:fa:81:a8:d8':
        print(line)
#     tmp +=1
#     if tmp==2:
#         break

# mac = line[0:17]
# rssi = line[23:26]

    