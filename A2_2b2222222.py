import cv2 as cv
import numpy as np
from time import sleep

# Setting up camera capture
cap = cv.VideoCapture(1)
cap.set(cv.CAP_PROP_FRAME_WIDTH,1000)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,1000)


given = int(input("Enter 1 to take pic: "))

if (given == 1):
    # Take the frame
    
    sleep(0.1)
    _, frame = cap.read()

#cv.imshow('frame',frame)
    
# Convert BGR to HSV
hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

# define range of green color in HSV
lower_green = np.array([40,100,100])
upper_green = np.array([80,255,255])

# Threshold the HSV image to get only green colors
mask = cv.inRange(hsv, lower_green, upper_green)

# Bitwise-AND mask and original image
res = cv.bitwise_and(frame,frame, mask= mask)

cv.imshow('Frame',frame)

contour,_ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
drawn = cv.drawContours(frame, contour, -1, (255,0,0), 3)



cv.imshow('mask',mask)
cv.imshow('res',res)
cv.imshow('Contour',drawn)

cv.waitKey(0)

cv.destroyAllWindows()

