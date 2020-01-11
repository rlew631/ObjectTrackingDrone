#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import functions
# for dronekit
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
# for opencv
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
# Set up option parsing to get connection string
import argparse
# definitions page
import definitions

# <editor-fold desc="input argument parser for dronekit">
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None
if not connection_string:  # Start SITL if no connection string specified
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
print('Connecting to vehicle on: %s' % connection_string)  # Connect to the Vehicle
vehicle = connect(connection_string, wait_ready=True)
# </editor-fold>
# <editor-fold desc="input argument parser for opencv">
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())


# </editor-fold>

# global variable(s)
x_max = 1024  # width of pi camera image
y_max = 768   # height of pi camera image
x_center = x_max / 2  # center value used for readability/ease of editing
y_center = y_max / 2  # center value used for readability/ease of editing


def object_tracking(x, y):
    global x_center  # importing the global x_center variable
    global y_center  # importing the global y_center variable
    x_error = x - x_center  # calculating the x error in the distance of the ball from the center of the image
    y_error = y - y_center  # calculating the x error in the distance of the ball from the center of the image
    print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))
    print("X error: ".format(x_error))
    print("Y error: ".format(y_error))
    if abs(x_error) > 20:
        condition_yaw(x_error/10, relative=True)
    if abs(y_error) > 20:
        condition_gimbal_pitch(y_error/10)


print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


print(" Ch8 (check this one...)      : %s" % vehicle.channels['8'])


# have it so the flight modes run indefinitely and have a break in them which happens when channel 7 is switched
while vehicle.channels['8'] > 20:
    arm_and_takeoff(1)  # Arm and take of to altitude of 1 meter
    print(" Ch7 (check this one...)      : %s" % vehicle.channels['7'])
    if (vehicle.channels['7'] >= 100) & (vehicle.channels['7'] < 200):
        # normal flight, see if there's anything (channel override??) that needs to be undone after guided mode
    if (vehicle.channels['7'] >= 200) & (vehicle.channels['7'] < 300):
        # guided flight, do something with channel overrride here
        object_detection()


# The example is completing. LAND at current location.
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
