
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

degPerPixel= 0.04664
q=queue.Queue()

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
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

	
# Let the camera warmup
sleep(0.1)
	
# Get an image from the camera stream
ret, image = camera.read()
dummy=""
last_state=0
	
if not ret:
	print("Could not capture image from camera!")
	quit()
else:
    while(1):
        
        grey=cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
        overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2BGR) # Convert back to RGB for imshow, as well as for the next step
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
        if ids is None:
            q.queue.clear()
        if not ids is None:
                FocalLength=(100*abs(corners[0][0][3][0]-corners[0][0][2][0])/6.6)
                distance = (1091 * 6.6) / abs(corners[0][0][3][0]-corners[0][0][2][0])
                #print(f"distance is {distance}")

                angle= degPerPixel *((corners[0][0][3][0]+corners[0][0][2][0])/2 - 640)
                print(f"Angle is {angle}")
                q.put(angle)
                for (outline, id) in zip(corners, ids):
                        markerCorners = outline.reshape((4,2))
                        value=str(ids)
                        if(value not in dummy): 
                            dummy+=value
                        overlay = cv2.putText(image, str(id),(int(markerCorners[0,0]), int (markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0,0), 2)
                

                        
        cv2.imshow("overlay",image)

        ret, image = camera.read()
        k=cv2.waitKey(1) & 0xFF
        if k== ord('q'):
                break
