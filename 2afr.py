
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_6X6_250)


# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear()
lcd.color = [0, 0, 100]

	
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
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
        overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2RGB) # Convert back to RGB for imshow, as well as for the next step
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
        if not ids is None:
                if last_state==0:
                    lcd.clear()
                ids = ids.flatten()
                
                print("The id is:", str(id))
                for (outline, id) in zip(corners, ids):
                        markerCorners = outline.reshape((4,2))
                        value=str(ids)
                        if(value not in dummy): 
                            dummy+=value
                        overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int (markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0,0), 2)
                        lcd.color= [0,100,0]
                        lcd.message = ("There are marker[s]:\n"+dummy )
                dummy=""
                last_state=1
                        
        cv2.imshow("overlay",overlay)
        if ids is None:
            if last_state==1:
                lcd.clear()
            lcd.message=("No id is found")
            lcd.color=[100,0,0]
            last_state=0
            
        ret, image = camera.read()
        k=cv2.waitKey(1) & 0xFF
        if k== ord('q'):
                break

lcd.clear()
lcd.color=[0,0,0]
