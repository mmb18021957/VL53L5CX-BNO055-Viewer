
import network
import espnow
import struct
import array

from wifi_connect import *
import socket

from machine import I2C, Pin
from bno055 import *
i2c = I2C(1, scl=Pin(17), sda=Pin(16))
imu = BNO055(i2c,0x29)

from vl53l5cx.mp import VL53L5CXMP

scl_pin, sda_pin, lpn_pin, _ = (22, 21, 18, 19)
i2c = I2C(0, scl=Pin(scl_pin, Pin.OUT), sda=Pin(sda_pin), freq=1_000_000)
tof = VL53L5CXMP(i2c, lpn=Pin(lpn_pin, Pin.OUT, value=1))
from sensor import make_sensor

from vl53l5cx import DATA_TARGET_STATUS, DATA_DISTANCE_MM
from vl53l5cx import STATUS_VALID, RESOLUTION_8X8

# WLAN initialisieren (ESP-NOW benÃƒÂ¶tigt aktives WLAN)
sta = network.WLAN(network.STA_IF)
sta.active(True)


# MAC-Adresse des EmpfÃƒÂ¤ngers anpassen!
# 1C:DB:D4:3D:12:F0
e = espnow.ESPNow()
e.active(True)
peer = b'\x1C\xDB\xD4\x3D\x12\xF0'   # MAC servo

try:
    e.del_peer(peer)  # Will raise if peer not found, so wrap in try
except OSError:
    pass
e.add_peer(peer)

dist_arr = array.array('H', [0] * 64)
stat_arr = array.array('B', [0] * 64)
p_arr    = array.array('f', [0.0] * 4)
    
def main():
    
    ssid = 'VL53L5CX-BNO08X'
    password = '123456789'

    ip_s = '100.100.100.9'    

    port_pc  = 9999

    ip_s = create_accesspoint(ssid, password, ip_s)
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    calibrated = False

    tof = make_sensor()
    tof.reset()

    if not tof.is_alive():
        raise ValueError("VL53L5CX not detected")

    tof.init()

    tof.resolution = RESOLUTION_8X8
    grid = 7

    tof.ranging_freq = 15

    tof.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})
    
    if not calibrated:
        calibrated = imu.calibrated()

    while True:
                    
        data = ""          
        if tof.check_data_ready():
            results = tof.get_ranging_data()
            distance = results.distance_mm            
            status = results.target_status
            
            data += "{\"distances\":["
            
            for i, d in enumerate(distance):
                data += "{},".format(d)
                dist_arr[i] = d                
            if len(data) > 0:
                data = data[:-1]  
            
            data += "],\"status\":["
            for i, d in enumerate(distance):
                data += "{},".format(status[i])
                stat_arr[i] = status[i]
            if len(data) > 0:
                data = data[:-1]

            
        # BNO055-028    
        q = imu.quaternion()
        p_arr = [q[0],-q[1],-q[2],q[3]]
        
        data += '],\"quat\":['
        data += ("{:6},{:6},{:6},{:6}".format(q[0],-q[1],-q[2],q[3]))
        data += '],\"v\":\"'
        data += "0.1.0"
        data += '\"}\n\r'
        print(data)

        # Senden
        fmt = "<64H64B4f"   # < = little endian
        data_packed = struct.pack(fmt, *dist_arr, *stat_arr, *p_arr)
        try:
            e.send(peer, data_packed)
        except OSError:
            pass
        
        try:
            s.sendto(data.encode('utf-8'), ('255.255.255.255', port_pc))            
        except OSError as err:
            #print(data)
            pass
                    
main()