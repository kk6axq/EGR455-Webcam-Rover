"""
hsv_slider.py
October 25, 2022
Lukas Severinghaus, Carlos Chacon, Miguel Chacon, Salsabil Soliman

Filters a live image by hue, saturation, and value using sliders.
Features two hue sliders for use with red hue ranges.
Also has a slider to enable/disable a gaussian blur on the input image.

Displays live:
* The masked image result
* The binary mask
* The original input image

Used for determining filter settings for the EGR455 team project.
"""

import cv2
import imutils
import numpy as np

# Change this to the video input device number
# on Ubuntu, use ls /dev/video*    
cam = cv2.VideoCapture(3)

# Range variables
hsv_low = 0
hsv_high = 180
hsv2_low = 0
hsv2_high = 180
value_low = 0
value_high = 255
sat_low = 0
sat_high = 255

# Slider update functions
# Each of these updates one of the range variables when the slider is moved
def on_hsv_change_low(value):
    global hsv_low
    hsv_low = value
def on_hsv_change_high(value):
    global hsv_high
    hsv_high = value
    
def on_hsv2_change_low(value):
    global hsv2_low
    hsv2_low = value
def on_hsv2_change_high(value):
    global hsv2_high
    hsv2_high = value
    
def on_value_change_low(value):
    global value_low
    value_low = value
    
def on_value_change_high(value):
    global value_high
    value_high = value

def on_sat_change_low(value):
    global sat_low
    sat_low = value
    
def on_sat_change_high(value):
    global sat_high
    sat_high = value

# Callback that does nothing for the filter slider
def nothing_fn(v):
    pass
    
# Setup the windows
cv2.namedWindow("Mask")
cv2.namedWindow("Mask raw")
cv2.namedWindow("Raw")

# Create the trackbars and set their initial positions
cv2.createTrackbar('Hue Low', "Mask", 0, 180, on_hsv_change_low)
cv2.createTrackbar('Hue High', "Mask", 0, 180, on_hsv_change_high)
cv2.setTrackbarPos('Hue High', 'Mask', 180)
cv2.createTrackbar('Hue 2 Low', "Mask", 0, 180, on_hsv2_change_low)
cv2.createTrackbar('Hue 2 High', "Mask", 0, 180, on_hsv2_change_high)
cv2.setTrackbarPos('Hue 2 High', 'Mask', 180)
cv2.createTrackbar('Sat Low', "Mask", 0, 255, on_sat_change_low)
cv2.createTrackbar('Sat High', "Mask", 0, 255, on_sat_change_high)
cv2.setTrackbarPos('Sat High', 'Mask', 255)
cv2.createTrackbar('Value Low', "Mask", 0, 255, on_value_change_low)
cv2.createTrackbar('Value High', "Mask", 0, 255, on_value_change_high)
cv2.setTrackbarPos('Value High', 'Mask', 255)

# Create the blur switch
# OpenCV doesn't have a switch, so use a trackbar as a switch
blur_switch = 'Use blur (1=Yes, 0=No)'
cv2.createTrackbar(blur_switch, "Mask", 0, 1, nothing_fn)

# Based on https://stackoverflow.com/questions/54051094/color-blocks-detection-and-label-in-opencv
while True:
    # Read the image
    ret, img = cam.read()
    # Build the threshold brackets
    lower_val = (hsv_low, sat_low, value_low)
    upper_val = (hsv_high, sat_high, value_high)
    lower2_val = (hsv2_low, sat_low, value_low)
    upper2_val = (hsv2_high, sat_high, value_high)
    
    # Add blur if it's enabled 
    if cv2.getTrackbarPos(blur_switch, "Mask") == 1:
        img = cv2.GaussianBlur(img, (5, 5),5)
        
    # convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    
    # Mask with the two thresholds and combine them
    mask = cv2.inRange(hsv, lower_val, upper_val)
    mask2 = cv2.inRange(hsv, lower2_val, upper2_val)
    mask_new = cv2.bitwise_or(mask, mask2)
    
    # Build the masked image
    masked_img = cv2.bitwise_and(img, img, mask=mask_new)
    
    # Display all the images
    cv2.imshow("Mask raw", mask2)
    cv2.imshow("Mask", masked_img)
    cv2.imshow("Raw", img)
    
    # Continue looping
    cv2.waitKey(1)
