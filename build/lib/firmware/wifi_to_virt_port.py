import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port_pc = 9999

import serial
port = "COM10"

ser = serial.Serial(
    port=port,
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1
)

def main():
    try:
        s.bind(('0.0.0.0', port_pc))
    except socket.error:
        return

    while True:
        try:
            s.settimeout(0.1)
            raw = s.recv(1024).decode()
            print(raw)
        
            try:
                #s.settimeout(0.1)
                ser.write(raw.encode())
                ser.flush()
            except:
                print("seriell error")
                pass

        except socket.timeout:
            pass

if __name__ == "__main__":
    main()