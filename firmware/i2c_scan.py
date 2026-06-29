import machine

i2c_1 = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
device_1 = i2c_1.scan()
i2c_2 = machine.I2C(scl=machine.Pin(17), sda=machine.Pin(16))
device_2 = i2c_2.scan()

if len(device_1) == 0:
    print("No I2C device_1 found!")
else:
    print('I2C devices found:', len(device_1))
for device in device_1:
    print("Decimal address:", device, "| Hexa address:", hex(device))

if len(device_2) == 0:
    print("No I2C device_2 found!")
else:
    print('I2C device_2 found:', len(device_2))
for device in device_2:
    print("Decimal address:", device, "| Hexa address:", hex(device))
