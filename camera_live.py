"""
camera_live.py
October 25, 2022
Lukas Severinghaus, Carlos Chacon, Miguel Chacon, Salsabil Soliman

Uses OpenCV to localize a robot and target, then performs path planning and state logic
to control the rover with TCP commands. Used for the EGR455 project 2.
"""





import cv2
import imutils
import numpy as np
import math
import time
import socket
import struct

# Color threshold dictionary, with high low pairs
# Order: "name": [Hue 1 low, Hue 1 high, Hue 2 low, Hue 2 high, value low, value high, saturation low, saturation high]
threshold_values = {
    "red": [168, 180, None, None, 52, 149, 100, 255],
    "green": [45, 101, None, None, 60, 168, 74, 255],
    "blue": [103, 148, None, None, 63, 129, 70, 255]
}

# Calculate the center position given three color circle tuple positions
# This is just an arithmetic mean of x and y positions
# Returns a tuple (x, y) mean position
def center_pos(r, g, b):
    return int((r[0] + g[0] + b[0]) / 3), int((r[1] + g[1] + b[1]) / 3)
    
# Calculate the pythagorean distance between two tuples
# Returns a float distance
def calc_dist(a, b):
    return math.sqrt(math.pow(a[0] - b[0], 2) + math.pow(a[1] - b[1], 2))
    
# Calculate the bearing between two positions, in degrees
def bearing(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0]) / math.pi * 180


# color_blob: finds the position of a blob of a given color, mainly sorting on hue. 
# This function is designed to isolate the three circles on the robot, one at a time.
# It returns an (x, y) coordinate pair denoting the location of the robot, or throws an exception if it can't find one.
# Based on https://stackoverflow.com/questions/54051094/color-blocks-detection-and-label-in-opencv
# This function throws a BufferError if there's an issue acquiring a solution
# I know BufferError is the wrong exception for this, but we had issues with a plain exception getting thrown by actual error conditions,
# so we needed to differentiate our exceptions from the actual issues.
def color_blob(img, packed_thresholds, title, show=False, debug=False):
    # Unpack the threshold array into individual values
    hsv_low = packed_thresholds[0]
    hsv_high = packed_thresholds[1]
    second_low = packed_thresholds[2]
    second_high = packed_thresholds[3]
    value_low = packed_thresholds[4]
    value_high = packed_thresholds[5]
    # Unpack saturation values if they're present, otherwise use default values
    if len(packed_thresholds) > 6:
        sat_low = packed_thresholds[6]
        sat_high = packed_thresholds[7]
    else:
        sat_low = 100
        sat_high = 255
    
    # Add image blur, to reduce noise
    img = cv2.GaussianBlur(img, (5, 5),5)
    
    # convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    
    # set lower and upper color limits
    lower_val = (hsv_low, sat_low, value_low)
    upper_val = (hsv_high,sat_high,value_high)
    
    # Second set of color limits, for use with red
    second_val_low = (second_low, sat_low, value_low)
    second_val_high = (second_high, sat_high, value_high)
    
    # Threshold the HSV image to get only in range colors
    mask = cv2.inRange(hsv, lower_val, upper_val)
    
    # Use the second filter if it's set up
    if second_low is not None:
        #print("Using second filter")
        mask_2 = cv2.inRange(hsv, second_val_low, second_val_high)
        mas = cv2.bitwise_or(mask, mask_2)
        
    # apply mask to original image
    res = cv2.bitwise_and(img,img, mask= mask)
    
    
    #cv2.imshow("Result", res)
    
    # detect contours in image
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    

    if debug: print("Found {} contours".format(len(contours)))

    # Average the position of any valid contours found, to find the center of the blob
    sum_x = 0
    sum_y = 0
    sum_len = len(contours)
    for cnt in contours:
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            sum_len = sum_len - 1
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        sum_x = sum_x + cX
        sum_y = sum_y + cY
        if debug: print("Position: {}, {}".format(cX, cY))
        cv2.drawContours(res, [cnt], 0, (0,0,255), 2)
    
    # Check if no blobs found
    if sum_len == 0:
        raise BufferError("No blobs found: " + title)
    sum_x = sum_x / sum_len
    sum_y = sum_y / sum_len 
    if debug: print("Root position: {}, {}".format(sum_x, sum_y))
    
    # detect edges in mask
    edges = cv2.Canny(mask,100,100)
    
    #show images
    #cv2.imshow("Result_with_contours", res)
    
    if show: cv2.imshow(title, mask)
    #cv2.imshow("Edges", edges)
    # Return the integer x, y position of the area in pixel space
    return int(sum_x), int(sum_y)

# value_blob: This function tries to find a blob's location based on it's HSV value, and is designed for locating the target
# it also does a bit of filtering to find a blob that is the size of the target, rejecting larger and smaller items.
# It also considers the robot position when locating a blob, using distance to potential blobs to help prevent detecting the robot body as the target.
# It also considers the bounding circle of the blob, to reject long slender blobs, like off the edge of the table.
# Based on https://stackoverflow.com/questions/54051094/color-blocks-detection-and-label-in-opencv
# This function throws a BufferError if there's an issue acquiring a solution
# I know BufferError is the wrong exception for this, but we had issues with a plain exception getting thrown by actual error conditions,
# so we needed to differentiate our exceptions from the actual issues.
def value_blob(img, value_low, value_high, title, robot_pos, show=False, debug=False):
    # Add blur to reduce noise
    img = cv2.GaussianBlur(img, (5, 5),5)
    # convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    # set lower and upper color limits
    lower_val = (0, 0, value_low)
    upper_val = (180,255,value_high)
    # Threshold the HSV image to get only the in range pixels
    mask = cv2.inRange(hsv, lower_val, upper_val)

    # apply mask to the original image
    res = cv2.bitwise_and(img,img, mask= mask)
    
    #cv2.imshow("Result", res)
    
    # detect contours in image
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if debug: print("Found {} contours".format(len(contours)))

    # Find contours and record their positions and distances from the robot
    possible_dists = []
    possible_positions = []
    for cnt in contours:
        # Calculate the area of the target
        cnt_size = cv2.contourArea(cnt)
        
        # Skip contours that are not the right size
        if cnt_size > 2000 or cnt_size < 1200:
            continue
        
        # Calculate the bounding circle to determine rough size/shape
        bound_center, bound_radius = cv2.minEnclosingCircle(cnt)
        
        # Reject contours with large bounding radius
        if bound_radius > 50:
            continue
        if debug: print("Bounding circle: ", bound_radius)
        if debug: print("Size: ", cnt_size)
        
        # Calculate the position of the contour
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
            
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # Calculate distance from the centroid to the robot
        dist = math.sqrt(math.pow(cX - robot_pos[0], 2) + math.pow(cY - robot_pos[1], 2))
        
        # Append it to the possible positions arrays
        possible_dists.append(dist)
        possible_positions.append([cX, cY])
        if debug: print("Position: {}, {}".format(cX, cY))
        
        cv2.drawContours(res, [cnt], 0, (0,0,255), 2)
    
    # No contours found
    if len(possible_dists) == 0:
        raise BufferError("No target found")
    if debug: print("Dists: ", possible_dists)
    if debug: print("Pos: ", possible_positions)
    
    # Find the farthest away blob
    max_index = possible_dists.index(max(possible_dists))
    target_pos = possible_positions[max_index]
    
    if debug: print("Target pos: ", target_pos)
    
    
    # detect edges in mask
    edges = cv2.Canny(mask,100,100)
    
    #cv2.imshow("Result_with_contours", res)
    if show: cv2.imshow(title, mask)
    
    #cv2.imshow("Edges", edges)
    
    # Return the integer x, y coordinate position, in pixel space
    return int(target_pos[0]), int(target_pos[1])


# Calculates the movement vector given the input image. 
# First locates the robot and target, then calculates the bearing and distance. 
# Does some filtering to validate the positions.
# Handles conditions where the robot or target is obscured.
def calculate_vector(input_img, show=False, debug=False):
    
    # Try to get the RGB circle positions, if not, return a null value
    try:
        green_pos = color_blob(input_img, threshold_values['green'], "Green")
        blue_pos = color_blob(input_img, threshold_values['blue'], "Blue")
        red_pos = color_blob(input_img, threshold_values['red'], "Red")
        #print("RGB pos: ", red_pos, green_pos, blue_pos)
    except BufferError as e:
        print(e)
        raise ValueError("Missing a sticker")
        return None, None, None
    
    # Check if the color circles are too far away, if so reject the position solution
    if calc_dist(green_pos, blue_pos) > 100 or calc_dist(green_pos, red_pos) > 100 or calc_dist(red_pos, blue_pos) > 100:
        return None, None, None
    
    # Calculate the robot's center position
    robot_center = center_pos(red_pos, green_pos, blue_pos)
    
    # print("Robot center: ", robot_center)

    # Calculate the target position, or return a null value if it can't be found
    try:
        # Calculate target position
        target_pos = value_blob(input_img, 0, 70, "Black", robot_center, show=False)
    except BufferError as e:
        raise BufferError("No target")
        print(e)
        return None, None, None
    
    # Calculate the distance and absolute bearing to the target
    target_dist = calc_dist(robot_center, target_pos)
    target_bearing = bearing(robot_center, target_pos)

    # Calculate the heading of the robot
    # The robot heading is the vector from the center of the robot through the red circle
    # print("Target pos: ", target_pos)
    heading_angle = bearing(robot_center, red_pos)

    # Calculate the relative bearing to target
    delta_theta = target_bearing - heading_angle

    # Draw a line at the robot heading
    heading_line = (robot_center[0] + 100 * math.cos(heading_angle*math.pi/180), robot_center[1] + 100 * math.sin(heading_angle*math.pi/180))
    heading_line = [int(a) for a in heading_line]
    # print(heading_line)
    #print("Target bearing: ", target_bearing)
    if debug: print("Center: ", robot_center)
    # print("Heading angle: ", heading_angle)
    if debug: print("\n\n\n****")
    if debug: print("Dist: ", target_dist)
    # print("Delta Theta: ", delta_theta)
    # Draw the position circles, target line, and heading line
    plot_image = input_img.copy()
    plot_image = cv2.circle(plot_image, green_pos, radius=5, color=(0, 255, 0), thickness=1)
    plot_image = cv2.circle(plot_image, red_pos, radius=5, color=(0, 0, 255), thickness=1)
    plot_image = cv2.circle(plot_image, blue_pos, radius=5, color=(255, 0, 0), thickness=1)
    plot_image = cv2.circle(plot_image, robot_center, radius=5, color=(255, 255, 255), thickness=1)
    plot_image = cv2.circle(plot_image, target_pos, radius=5, color=(255, 255, 255), thickness=1)
    plot_image = cv2.line(plot_image, robot_center, target_pos, (255, 0, 0), 3)
    plot_image = cv2.line(plot_image, robot_center, heading_line, (0, 255, 0), 3)
    plot_image = cv2.line(plot_image, (100, 100), (200, 100), (0, 0, 255), 3)
    plot_image = cv2.line(plot_image, (100, 100), (100, 300), (0, 0, 255), 3)
    if show:
        cv2.imshow("Result", plot_image)
    # Return the heading, distance, and image with annotations
    return delta_theta, target_dist, plot_image


# Old attempt to pull the video from a thread, to reduce buffer issues
"""        
img_ready = False
latest_frame = None
def cap_thread(name):
    global latest_frame, img_ready
    cam = cv2.VideoCapture(name)
    while True:
        img_ready = True
        ret, latest_frame = cam.read()
        
"""

# Initialize the TCP connection
def init_tcp():
    global tcp_ang, tcp_vel, tcp_fork
    host = '192.168.0.134'
    # Service ports
    ang_port = 25002
    fork_port = 25001
    vel_port = 25000
    # Open the socket connections
    tcp_fork = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_fork.connect((host, fork_port)) 
    tcp_ang = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_ang.connect((host, ang_port))
    tcp_vel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_vel.connect((host, vel_port))
    

# Close the socket connections
def close_tcp():
    global tcp_ang, tcp_vel, tcp_fork
    tcp_ang.close()
    tcp_vel.close()
    tcp_fork.close()

# Helper functions to encode and send values for linear velocity, angular velocity, and fork position
def send_theta(value):
    global tcp_ang
    data = struct.pack("<d", value)
    tcp_ang.sendall(data)
    print("Sent bytes")
    
def send_vel(value):
    global tcp_vel
    data = struct.pack("<d", value)
    tcp_vel.sendall(data)
    print("Sent bytes")
    
def send_fork(value):
    global tcp_fork
    data = struct.pack("<d", value)
    tcp_fork.sendall(data)
    print("Sent bytes fork")
 

init_tcp()

#cv2.namedWindow("Result")
# Start video capture
cam = cv2.VideoCapture(2) # Was device 5

# State machine variables
can_turn = True
approached = False
fork_override = False

# Forklift positions
FORK_DOWN = 120
FORK_UP = 180

try:
    send_fork(FORK_DOWN)
    while True:
        # Get the image
        ret, img = cam.read()
        plot_img = None
        theta = None
        dist = None
        # Try to calculate the movement vector
        try:
            theta, dist, plot_img = calculate_vector(img, show=False)
        except ValueError:
            send_theta(0)
            send_vel(0)
            print("Stopping")
        except BufferError:
            # If the target was lost, then pick up the forks as the target has probably merged with the body blob
            if approached:
                send_fork(FORK_UP)
                send_vel(0)
                send_theta(0)
                print("Forks up, target lost")
                fork_override = True

        if plot_img is not None:
            cv2.imshow("Result", plot_img)
                
        if None in [theta, dist]:
            print("Skip due to no position solution")
            continue
            
        print("Can turn: ", can_turn)
        print("Completed final approach: ", approached)
        print("Fork override: ", fork_override)
        # Different angle thresholds if far away or in final approach
        approach_angle_threshold = 3 if not approached else 1
        # Turning mode
        if abs(theta) > approach_angle_threshold and can_turn:
            print("Mode turning")
            if theta < -180:
                theta = theta + 360
            if theta > 180:
                theta = theta - 360
            if theta > 0:
                print("Turn right ", theta)
                send_theta(-2)
                time.sleep(0.005)
                send_theta(0)
                time.sleep(0.005)
            else:
                print("Turn left", theta)
                send_theta(2)
                time.sleep(0.005)
                send_theta(0)
                time.sleep(0.005)
        else: # Linear movement mode
            if not fork_override:    
                if approached and False:
                    cv2.imshow("Approach", plot_img)
                print("Mode move forward")
                print("Dist: ", dist)
                send_theta(0)
                can_turn = False
                if dist < 300 and not approached:
                    can_turn = True
                    approached = True
                if dist > 130:
                    
                    send_vel(8)
                    print(dist)
                    print("Go forward")
                else:
                    approached = False
                    print("We're here")
                    send_fork(FORK_UP)
                    send_vel(0)
            
        # Lower the fork if the target is far away
        if dist > 250:
            send_fork(FORK_DOWN)
        
        # Reset the approach mode if the target is far away
        # This lets it pick up multiple times after the target is moved again
        if dist > 325:
            approached = False
            fork_override = False
            can_turn = True
            #print("Clear state")

        cv2.waitKey(1)
        
# When Ctrl-C pressed, close the camera and TCP connections
except KeyboardInterrupt:
    cam.release()
    close_tcp()
    print("Quitting")
    cv2.destroyAllWindows()
