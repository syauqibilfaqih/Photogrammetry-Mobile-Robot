import serial
import numpy as np
import time


with open('/home/pi/MobileRobotCommands/speed.txt') as f:
    speed = f.read()

with open('/home/pi/MobileRobotCommands/port.txt') as f:
    port = f.read()

with open('/home/pi/MobileRobotCommands/leftspeedscale.txt') as f:
    leftspeedscale = f.read()

with open('/home/pi/MobileRobotCommands/rightspeedscale.txt') as f:
    rightspeedscale = f.read()

ser = serial.Serial(port, 115200, timeout=1)



data = np.zeros(12, dtype=np.uint8)

numofpict=0
radoftrack = 0
pwmL    = int(int(speed)*float(leftspeedscale))
pwmR    = int(int(speed)*float(rightspeedscale))

#hex of '3' 'd' 'p' 0x33 0x64 0x70
data[0] = 0x33
data[1] = 0x64
data[2] = 0x70
data[3] = 0
data[4] = 0 & 0xFF
data[5] = 0 >> 8
data[6] = radoftrack & 0xFF
data[7] = radoftrack >> 8
data[8] = pwmL & 0xFF
data[9] = pwmL >> 8
data[10] = pwmR & 0xFF
data[11] = pwmR >> 8

ser.write(data)
ser.close()
