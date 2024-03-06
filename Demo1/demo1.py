#Max Allen and Tate Morrison
#This code does the following:
#Loads our calibrated Matrices
#Takes video feed of calibrated photo
#Checks for Aruco's
#Calculates Angle of detected

from numpy import savetxt
from numpy import genfromtxt
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus
import time
import board
import queue
import threading
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#Loading in previously calculated matrix undistortion
rvecs=genfromtxt('/home/seedlab/CalibrationImages/rotation_vecs.csv',delimiter=',')
tvecs=genfromtxt('/home/seedlab/CalibrationImages/translation_vecs.csv',delimiter=',')
mtx=genfromtxt('/home/seedlab/CalibrationImages/camera_matrix.csv',delimiter=',')
dist=genfromtxt('/home/seedlab/CalibrationImages/camera_distortion.csv',delimiter=',')
degPerPixel= 0.04664
q=queue.Queue()

#Writing to the LCD board and using threading to be efficient
def LCDWrite():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    i2d=SMBus(1)
    ADR=8;
    lcd_columns = 16
    lcd_rows = 2
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

    lcd.clear()
    lcd.color = [0, 0, 100]
    while True:
        if not q.empty():
            something=q.get()
            q.queue.clear()
            something=round(something,4)
            something=str(something)
            lcd.color= [0,100,0]
            lcd.message = ("The angle is: \n"+something)
#            command=[ord(character) for character in something]
#            i2d.write_i2c_block_data(ADR,0,command)
myThread=threading.Thread(target=LCDWrite,args=())
myThread.start()

aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# initialize the camera. If channel 0 doesn't work, try channel 1
# Setting cam properties and auto exposure
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE,.25)
camera.set(cv2.CAP_PROP_EXPOSURE, -7)
	
# Let the camera warmup
sleep(0.1)
	
# Get an image from the camera stream
ret, image = camera.read()

#Find calibration matrix for this image
h,w =image.shape[:2]
newcameramtx,roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

#Checking if cam took photo
if not ret:
	print("Could not capture image from camera!")
	quit()
else:
    # Cam succeeded
    while(1):
        # Camera calibration / undistorton
        dst=cv2.undistort(image,mtx,dist,None,newcameramtx)
        x,y,w,h=roi
        dst=dst[y:y+h,x:x+w]

        # Detection of AruCo markers from greyscale
        grey=cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)

        #Commented out -- These lines show video feed
        #overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2BGR) # Convert back to RGB for imshow, as well as for the next step
        #overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)

        #Feeding to LCD
        if ids is None:
            q.queue.clear()
        if not ids is None:
                # Angle and distance calculations
                FocalLength=(100*abs(corners[0][0][3][0]-corners[0][0][2][0])/6.6)
                distance = (1091 * 6.6) / abs(corners[0][0][3][0]-corners[0][0][2][0])
                #print(f"distance is {distance}")

                angle= degPerPixel *((corners[0][0][3][0]+corners[0][0][2][0])/2 - 640)
                print(f"Angle is {angle}")
                q.put(angle)

                #Commented out -- These lines show video feed
                #for (outline, id) in zip(corners, ids):
                #        markerCorners = outline.reshape((4,2))
                #       value=str(ids)
                #        overlay = cv2.putText(dst, str(id),(int(markerCorners[0,0]), int (markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0,0), 2)
                

        #Commented out -- These lines show video feed              
        #cv2.imshow("overlay",dst)

        ret, image = camera.read()

        #Commented out -- These lines show video feed
        #k=cv2.waitKey(1) & 0xFF
        #if k== ord('q'):
        #        break
