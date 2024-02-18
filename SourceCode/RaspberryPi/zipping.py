import zipfile
import glob
import shutil

with zipfile.ZipFile('/home/pi/takenpictures.zip', 'w') as f:
    for file in glob.glob('/home/pi/takenPictures/*'):
        f.write(file)

src_path = "/home/pi/takenpictures.zip"
dst_path = "/home/pi/takenPictures/takenpictures.zip"
shutil.move(src_path, dst_path)
