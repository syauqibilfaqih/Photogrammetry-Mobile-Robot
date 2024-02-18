import os
 
dir_path = '/home/pi/takenPictures'
count =0 
for path in os.listdir(dir_path):
    # check if current path is a file
    if os.path.isfile(os.path.join(dir_path, path)):
        count += 1

x=1
while x<count+1:
    os.remove('/home/pi/takenPictures/'+str(x)+'.jpg')
    x=x+1
 
