import network
import espnow
import struct
import array
#mac 1C:DB:D4:3D:12:F0
sta = network.WLAN(network.STA_IF)
sta.active(True)

e = espnow.ESPNow()
e.active(True)

fmt = "<64H64B4f"   # < = little endian

dist_arr = array.array('H', [0] * 64)
stat_arr = array.array('B', [0] * 64)
p_arr    = array.array('f', [0.0] * 4)

while True:
    mac, msg = e.irecv()
    values = struct.unpack(fmt, msg)

    dist_arr = values[0:64]
    stat_arr  = values[64:128]
    p_arr   = values[128:132]
    
    data = ""                      
    data += "{\"distances\":["
    
    for i, d in enumerate(dist_arr):
        data += "{},".format(d)
    if len(data) > 0:
        data = data[:-1]  
    
    data += "],\"status\":["
    for i, d in enumerate(stat_arr):
        data += "{},".format(d)
    if len(data) > 0:
        data = data[:-1]
    
    data += '],\"quat\":['
    data += ("{:6},{:6},{:6},{:6}".format(p_arr[0],p_arr[1],p_arr[2],p_arr[3]))
    data += '],\"v\":\"'
    data += "0.1.0"
    data += '\"}\n\r'
    print(data)
