
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
            lcd.color= [0,0,100]
            lcd.message = ("The wheels are: \n" +something)
            command=[ord(character) for character in something]
            i2d.write_i2c_block_data(ADR,0,command)
            q.queue.clear()
myThread=threading.Thread(target=LCDWrite,args=())
myThread.start()
aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_6X6_250)


wheels="00"
last_wheels="99"

# Modify this if you have a different sized Character LCD
#lcd_columns = 16
#lcd_rows = 2

# Initialise I2C bus.
#i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialise the LCD class
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#lcd.clear()
#lcd.color = [0, 0, 100]

	
# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

	
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
        cv2.line(image,(320,0),(320,480),(255,255,0),2)
        cv2.line(image,(0,240),(640,240),(255,255,0),2)
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
        overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2BGR) # Convert back to RGB for imshow, as well as for the next step
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
        if ids is None:
            q.queue.clear()
        if not ids is None:
                ids = ids.flatten()
                if corners[0][0][2][0]<320 and corners[0][0][2][1]<240: #NW
                    wheels="01"
                if corners[0][0][3][0]>320 and corners[0][0][3][1]<240: #NE
                    wheels="00"
                if corners[0][0][1][0]<320 and corners[0][0][1][1]>240: #SW
                    wheels="11"
                if corners[0][0][0][0]>320 and corners[0][0][0][1]>240: #SE
                    wheels="10"
                for (outline, id) in zip(corners, ids):
                        markerCorners = outline.reshape((4,2))
                        value=str(ids)
                        if(value not in dummy): 
                            dummy+=value
                        overlay = cv2.putText(image, str(id),(int(markerCorners[0,0]), int (markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0,0), 2)
                q.put(wheels)
                

                        
        cv2.imshow("overlay",image)

        ret, image = camera.read()
        k=cv2.waitKey(1) & 0xFF
        if k== ord('q'):
                break
