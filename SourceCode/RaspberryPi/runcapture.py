import serial
import ctypes
import numpy as np
import time
import cv2
import os
import pandas as pd
#import keyboard

with open('/home/pi/MobileRobotCommands/port.txt') as f:
    port = f.read()

with open('/home/pi/MobileRobotCommands/numofpict.txt') as f:
    numofpict = f.read()

with open('/home/pi/MobileRobotCommands/radoftrack.txt') as f:
    radoftrack = f.read()

with open('/home/pi/MobileRobotCommands/runningstate.txt') as f:
    runningstate = f.read()

with open('/home/pi/MobileRobotCommands/cameratype.txt') as f:
    cameratype = f.read()

class DynamicArray(object):
	'''
	DYNAMIC ARRAY CLASS (Similar to Python List)
	'''

	def __init__(self):
		self.n = 0 # Count actual elements (Default is 0)
		self.capacity = 1 # Default Capacity
		self.A = self.make_array(self.capacity)

	def __len__(self):
		"""
		Return number of elements stored in array
		"""
		return self.n

	def __getitem__(self, k):
		"""
		Return element at index k
		"""
		if not 0 <= k < self.n:
			# Check it k index is in bounds of array
			return IndexError('K is out of bounds !')

		return self.A[k] # Retrieve from the array at index k

	def append(self, ele):
		"""
		Add element to end of the array
		"""
		if self.n == self.capacity:
			# Double capacity if not enough room
			self._resize(2 * self.capacity)

		self.A[self.n] = ele # Set self.n index to element
		self.n += 1

	def insertAt(self, item, index):
		"""
		This function inserts the item at any specified index.
		"""

		if index < 0 or index > self.n:
			print("please enter appropriate index..")
			return

		if self.n == self.capacity:
			self._resize(2*self.capacity)

		for i in range(self.n-1, index-1, -1):
			self.A[i+1] = self.A[i]

		self.A[index] = item
		self.n += 1

	def delete(self):
		"""
		This function deletes item from the end of array
		"""

		if self.n == 0:
			print("Array is empty deletion not Possible")
			return

		self.A[self.n-1] = 0
		self.n -= 1

	def removeAt(self, index):
		"""
		This function deletes item from a specified index..
		"""

		if self.n == 0:
			print("Array is empty deletion not Possible")
			return

		if index < 0 or index >= self.n:
			return IndexError("Index out of bound....deletion not possible")

		if index == self.n-1:
			self.A[index] = 0
			self.n -= 1
			return

		for i in range(index, self.n-1):
			self.A[i] = self.A[i+1]

		self.A[self.n-1] = 0
		self.n -= 1

	def _resize(self, new_cap):
		"""
		Resize internal array to capacity new_cap
		"""

		B = self.make_array(new_cap) # New bigger array

		for k in range(self.n): # Reference all existing values
			B[k] = self.A[k]

		self.A = B # Call A the new bigger array
		self.capacity = new_cap # Reset the capacity

	def make_array(self, new_cap):
		"""
		Returns a new array with new_cap capacity
		"""
		return (new_cap * ctypes.py_object)()

pos = DynamicArray()
ori = DynamicArray()

if cameratype == "webcam":
    cam = cv2.VideoCapture(0)

if cameratype == "dslr":
    dir_path = '/home/pi/takenPictures'
    count =0 
    for path in os.listdir(dir_path):
        # check if current path is a file
        if os.path.isfile(os.path.join(dir_path, path)):
            count += 1

    x=1
    while x<count:
        if count == 1 or count == 0:
            break
        os.remove('/home/pi/takenPictures/'+str(x)+'.jpg')
        x=x+1

ser = serial.Serial(port, 115200, timeout=1)

data = np.zeros(12, dtype=np.uint8)

pwmL    = 00
pwmR    = 00
#numofpict = 1000

#hex of '3' 'd' 'p' 0x33 0x64 0x70
data[0] = 0x33
data[1] = 0x64
data[2] = 0x70
data[3] = 2 #taskNumber
data[4] = int(numofpict) & 0xFF
data[5] = int(numofpict) >> 8
data[6] = int(radoftrack) & 0xFF
data[7] = int(radoftrack) >> 8
data[8] = pwmL & 0xFF
data[9] = pwmL >> 8
data[10] = pwmR & 0xFF
data[11] = pwmR >> 8

ser.write(data)
captureState="0"
steps=0
sequence=0

while True:
    infoRobot=ser.in_waiting
    infoRobot=ser.readline(infoRobot)
    #infoRobot=infoRobot.decode("utf-8").rstrip()
    print(infoRobot)
    try:
        captureState = infoRobot.decode("utf-8").rstrip().split(',')[2]
        pos.insertAt(infoRobot.decode("utf-8").rstrip().split(',')[0],sequence)
        ori.insertAt(infoRobot.decode("utf-8").rstrip().split(',')[1],sequence)
        sequence=sequence+1
    except IndexError:
        captureState="0"
    
    if captureState=="1":#.split(',')[2]=="1":
        if cameratype == "webcam":
            time.sleep(1)
            cam.open(0)
            ret, image = cam.read()
            cv2.imwrite('/home/pi/takenPictures/'+str(steps+1)+'.jpg',image)
            cam.release()    
            print('/home/pi/takenPictures/'+str(steps+1)+'.jpg')
            captureState = "0"
            steps=steps+1
            data[3]=4
            ser.write(data)
            data[3]=2
            captureState="0"
            infoRobot.decode("utf-8").rstrip().split(',')[2]="0"
        if cameratype == "dslr":
            time.sleep(1)
            os.system('gphoto2 --filename=/home/pi/takenPictures/'+str(steps+1)+'.jpg --capture-image-and-download')
            captureState = "0"
            steps=steps+1
            data[3]=4
            ser.write(data)
            data[3]=2
            captureState="0"
            infoRobot.decode("utf-8").rstrip().split(',')[2]="0"
    with open('/home/pi/MobileRobotCommands/runningstate.txt') as f:
        runningstate = f.read()
    if runningstate == "stop":
        exit()
    if int(steps)>int(numofpict)-1:
        break

i=0
positionExel=[0]*len(pos)
orientationExel=[0]*len(ori)
while i<len(pos):
    positionExel[i]=pos[i]
    orientationExel[i]=ori[i]
    print(str(i+1)+". "+str(pos[i])+","+str(ori[i]))

    i=i+1


df = pd.DataFrame(
    {
        "Position": positionExel,
        "Orientation": orientationExel,
    }
)

df = df[["Position", "Orientation"]]

writer = pd.ExcelWriter("/home/pi/takenPictures/posandori.xlsx", engine="xlsxwriter")

df.to_excel(writer, sheet_name="Sheet1", startrow=1, header=False, index=False)

workbook = writer.book
worksheet = writer.sheets["Sheet1"]

(max_row, max_col) = df.shape

column_settings = [{"header": column} for column in df.columns]
worksheet.add_table(0, 0, max_row, max_col - 1, {"columns": column_settings})
worksheet.set_column(0, max_col - 1, 12)
writer.close()
ser.close()
