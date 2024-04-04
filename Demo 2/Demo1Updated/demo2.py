#Max Allen and Tate Morrison
#This code does the following:
#Loads our calibrated Matrices
#Checks for Aruco's
#Calculates Angle and distance of detected
#Sends message to Arduino encoded to 1 bit for angle and 2 for distance

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
rvecs = genfromtxt('/home/seedlab/CalibrationImages/rotation_vecs.csv',delimiter=',')
tvecs = genfromtxt('/home/seedlab/CalibrationImages/translation_vecs.csv',delimiter=',')
mtx = genfromtxt('/home/seedlab/CalibrationImages/camera_matrix.csv',delimiter=',')
dist = genfromtxt('/home/seedlab/CalibrationImages/camera_distortion.csv',delimiter=',')

#Important starting values
degPerPixel = 0.04924
aruco_marker_size_in_cm = 6.6

#Initializing
q = queue.Queue()
q2 = queue.Queue()


#Sending Data and using threading to be efficient
def Send():
    i2d = SMBus(1)
    ADR = 8;
    distance_bit1 = '0'
    distance_bit2 = '0'
    angle_bit = '0'
    sent_flag = 0;
    
    while True:
       
        #If ARUCO detected
        if not q.empty():

            #Angle get and clear
            angle = q.get()
            q.queue.clear()

            #distance get and clear
            distance = q2.get()
            q2.queue.clear()

            #angle state machine
            if angle >= -30 and angle <-22.5:#If angle is far away on left side
                angle_bit='1'
            elif angle >= -22.5 and angle <-15:#If angle is on left side
                angle_bit='2'
            elif angle >= -15 and angle <-7.5:#If angle is close on left side
                angle_bit='3'
            elif angle >= -7.5 and angle <-3:#If angle is very close on left side
                angle_bit='4'
            elif angle >= -3 and angle <=3:#If angle is spot on
                angle_bit='5'
            elif angle > 3 and angle <=7.5:#If angle is very close on right side
                angle_bit='6'
            elif angle > 7.5 and angle <=15:#If angle is close on right side
                angle_bit='7'
            elif angle > 15 and angle <=22.5:#If angle is on right side
                angle_bit='8'
            elif angle > 22.5 and angle <=30:#If angle is far on right side
                angle_bit='9'

            
            #distance state machine
            '''
            Splits 8 foot range between 99 given codes
            Arduino knows the tranlsation constant and decodes
            back to original distance
            '''
            for i in range(99):
                dmax = i * 2.5
                dmin = (i - 1) * 2.5
                if (distance >= dmin and distance <= dmax):
                    if (i < 10):
                        i = str(i)
                        distance_bit1 = 0
                        distance_bit2 = i[0]
                    else:
                        i = str(i)
                        distance_bit1 = i[0]
                        distance_bit2 = i[1]
                    i = int(i)
            
            print(f"Bits: {distance_bit1} + {distance_bit2}")

            #Combine bits to one message to send to Arduino
            message = distance_bit1 + distance_bit2 + angle_bit

            #Unicode translation and sending
            command=[ord(character) for character in message]
            i2d.write_i2c_block_data(ADR,0,command)
            
#Threading
myThread=threading.Thread(target=Send,args=())
myThread.start()

#Aruco dictionary
aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

#Initialize the camera
#Setting cam properties and auto exposure
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE,.25)
camera.set(cv2.CAP_PROP_EXPOSURE, -13)
	
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
    #Camera succeeded
    while(1):
        
        #Camera calibration / undistorton
        dst = cv2.undistort(image,mtx,dist,None,newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h,x:x+w]

        # Detection of AruCo markers from greyscale
        grey=cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)

        #COMMENTED OUT -- These lines show video feed
        '''
        overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2BGR) # Convert back to RGB for imshow, as well as for the next step
        #overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
        '''
        
        #Feeding to threader
        if ids is None:
            q.queue.clear()
            q2.queue.clear()
        if not ids is None:
                #Angle and distance calculations
                #FocalLength=(25*abs(corners[0][0][3][0]-corners[0][0][2][0])/6.6)
                #print(FocalLength)

                #Focal length for distance calculated to be about 1040
                distance = (1043 * aruco_marker_size_in_cm) / abs(corners[0][0][3][0]-corners[0][0][2][0])
                distance=round(distance,1)
                print(f"distance is {distance}")

                angle= degPerPixel *-((corners[0][0][3][0]+corners[0][0][2][0])/2 - 616)
                print(f"Angle is {angle}")
                q.put(angle)
                q2.put(distance)

                #Commented out -- These lines show video feed
                '''
                #for (outline, id) in zip(corners, ids):
                        #markerCorners = outline.reshape((4,2))
                        #value=str(ids)
                        #overlay = cv2.putText(dst, str(id),(int(markerCorners[0,0]), int (markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0,0), 2)
                '''

        #COMMENTED OUT -- These lines show video feed              
        #cv2.imshow("overlay",dst)

        ret, image = camera.read()


        #COMMENTED OUT -- These lines show video feed
        '''
        k=cv2.waitKey(1) & 0xFF
        if k== ord('q'):
           break
        '''
