#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This file contains the definitions to interface with:
    Dronekit to interface with the pixhawk and gimbal
    OpenCV to perform object detection
    Perform
'''

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
# for opencv
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse #migth also be needed for defining vehicle. not sure
import cv2
import imutils
# Set up option parsing to get connection string
import argparse
# definitions page
import definitions

#get the conenction string by running mavproxy.py and seeing what connection it uses by default
connection_string = '/dev/serial/by-id/usb-ArduPilot_fmuv2_1C0034000951353332373834-if00'

# variables for image processing
x_max = 1024  # width of pi camera image
y_max = 768   # height of pi camera image
x_center = x_max / 2  # center value used for readability/ease of editing
y_center = y_max / 2  # center value used for readability/ease of editing
gimbal_y_pos = -45 #goes from -90 to 0

# Definition for OpenCV object detection
def object_detection():  # needs to be modified so definition can be called as part of main function
    """
    # Range for lower red
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    # Range for upper range
    lower_red = np.array([170, 120, 70])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    # Generating the final mask to detect red color
    mask1 = mask1 + mask2
    """
    green_lower = (29, 86, 6)  # define the lower boundaries of the "green"
    green_upper = (64, 255, 255)  # define the upper boundaries of the "green"
    pts = deque(maxlen=args["buffer"])  # ball in the HSV color space, then initialize the list of tracked points

    if not args.get("video", False):  # if a video path was not supplied, grab the reference to the picam
        vs = VideoStream(usePiCamera=args["picamera"] > 0).start()
    else:  # otherwise, grab a reference to the video file
        vs = cv2.VideoCapture(args["video"])
    time.sleep(2.0)  # allow the camera or video file to warm up
    while True:  # keep looping
        frame = vs.read()  # grab the current frame
        frame = frame[1] if args.get("video", False) else frame  # handle the frame from VideoCapture or VideoStream
        if frame is None:  # if viewing video and did not grab frame, then reached end of video
            break
        frame = imutils.resize(frame, width=600)  # resize the frame
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)  # blur it
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # and convert it to the HSV color space

        mask = cv2.inRange(hsv, green_lower, green_upper)  # construct a mask for the color "green"
        mask = cv2.erode(mask, None, iterations=2)  # then perform a series of erosions
        mask = cv2.dilate(mask, None, iterations=2)  # and dilations to remove any small blobs left in the mask

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)  # find contours in the mask
        cnts = imutils.grab_contours(cnts)
        center = None  # and initialize the current (x, y) center of the ball

        if len(cnts) > 0:  # only proceed if at least one contour was found
            c = max(cnts, key=cv2.contourArea)  # find the largest contour in the mask
            ((x, y), radius) = cv2.minEnclosingCircle(c)  # then use it to compute minimum enclosing circle and centroid
            M = cv2.moments(c)  # calculate moments
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  # use moment to find centroid in x,y
            if radius > 10:  # only proceed if the radius meets a minimum size
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)  # draw the circle
                cv2.circle(frame, center, 5, (0, 0, 255), -1)  # draw the centroid
                object_tracking(int(x), int(y))  # update the list of tracked points

        pts.appendleft(center)  # update the points queue
        for i in range(1, len(pts)):  # loop over the set of tracked points
            if pts[i - 1] is None or pts[i] is None:  # if either of the tracked points are None, ignore them
                continue
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)  # otherwise, compute thickness of line
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)  # draw the connecting lines

        cv2.imshow("Frame", frame)  # show the frame to our screen
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):  # if the 'q' key is pressed, stop the loop
            break

    if not args.get("video", False):  # if we are not using a video file, stop the camera video stream
        vs.stop()
    else:  # otherwise, release the camera
        vs.release()
    cv2.destroyAllWindows()  # close all windows

def condition_gimbal_pitch(deg):
   #vehicle.flush() put in main code only once
   #^^^this doesn't need to run twice per pitch update
   msg = vehicle.message_factory.mount_control_encode(
           0, 1,    # target system, target component
           deg*100, # pitch is in centidegrees, goes from -90 to zero i think
           0, # roll
           0, # yaw is in centidegrees
           0 # save position
           )
   print("mavlink message is: " + str(msg))
   vehicle.send_mavlink(msg)
   vehicle.flush()


##### HERE IS WHERE THE DEFINITION FILE ENDS/ENDED AND MAIN BEGINS

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()


#input argument parser for opencv
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())


print('Connecting to vehicle on: %s' % connection_string)  # Connect to the Vehicle
vehicle = connect(connection_string, wait_ready=True)


def object_tracking(x,y): #x and y are the position from image processing
    global x_center  # importing the global x_center variable
    global y_center  # importing the global y_center variable
    x_error = x - x_center  # calculating the x error in the distance of the ball from the center of the image
    y_error = y - y_center  # calculating the x error in the distance of the ball from the center of the image
    if (gimbal_y_pos + y_error >= -90) and (gimbal_y_pos + y_error <= 0):
        gimbal_y_pos = gimbal_y_pos + y_error
    print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))
    print("X error: ".format(x_error))
    print("Y error: ".format(y_error))
    #if abs(x_error) > 20:
    #    condition_yaw(x_error/10, relative=True)
    if (abs(y_error) > 20):
        condition_gimbal_pitch(gimbal_y_pos)
    print("Gimbal pitch: " + str(gimbal_y_pos))



vehicle.flush() #basically a clear all command

while True == True:
    object_detection()

#have it so the flight modes run indefinitely and have a break in them which happens when channel 7 is switched

'''
while vehicle.channels['8'] > 20:
    arm_and_takeoff(1)  # Arm and take of to altitude of 1 meter
    print(" Ch7 (check this one...)      : %s" % vehicle.channels['7'])
    if (vehicle.channels['7'] >= 100) & (vehicle.channels['7'] < 200):
        # normal flight, see if there's anything (channel override??) that needs to be undone after guided mode
    if (vehicle.channels['7'] >= 200) & (vehicle.channels['7'] < 300):
        # guided flight, do something with channel overrride here
        object_detection()
'''

'''
# The example is completing. LAND at current location.
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
'''

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

print("Completed")