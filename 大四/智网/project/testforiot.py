import bluetooth
import bluetooth._bluetooth as bluez 
import sys
dev_id = 0

try:
    sock = bluez.hci_open_dev(dev_id)
except:
    print('Error accessing bluetooth device.')
    sys.exit(1)

pkt = sock.recv(255)
ptype, event, plen = struct.unpack("BBB", pkt[:3])
print("Event: {}".format(event))


if event == bluez.EVT_INQUIRY_RESULT_WITH_RSSI:
    pkt = pkt[3:]
    nrsp = bluetooth.get_byte(pkt[0])
    for i in range(nrsp):
        addr = bluez.ba2str(pkt[1+6*i:1+6*i+6])
        rssi = bluetooth.byte_to_signed_int(
               bluetooth.get_byte(pkt[1 + 13 * nrsp + i]))
        results.append((addr, rssi))
        print("[{}] RSSI: {}".format(addr, rssi))

# https://github.com/pybluez/pybluez/blob/d7f36702bba2abc8ea41977b2eea56a17431ac6d/examples/advanced/inquiry-with-rssi.py#L18

# https://www.icode9.com/content-1-572771.

# https://www.cnblogs.com/wuyida/archive/2013/03/27/6300007.html

# https://github.com/ewenchou/bluetooth-proximity/blob/master/bt_proximity/bt_rssi.py

