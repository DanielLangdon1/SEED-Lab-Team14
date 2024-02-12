import cv2 as cv
import numpy as np
from time import sleep

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,480)

sleep(0.1)

_, image = cap.read()


# Convert BGR to HSV
hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

# define range of green color in HSV
lower_green = np.array([50,100,100])
upper_green = np.array([80,255,255])

# Threshold the HSV image to get only green colors
mask = cv.inRange(hsv, lower_green, upper_green)

# Bitwise-AND mask and original image
res = cv.bitwise_and(image,image, mask= mask)

# Making contour
# contour = cv.findContours(image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

cv.imshow('Picture',image)
cv.imshow('Mask',mask)
cv.imshow('Res',res)
# cv.imshow('Contour',contour)

# cv.destroyAllWindows()
