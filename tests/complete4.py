#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import functions for opencv
from __future__ import print_function
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import math

# variables for camera/gimbal settings
x_max = 600  # width of pi camera image
y_max = 450   # height of pi camera image
x_center = x_max / 2  # center value used for readability/ease of editing
y_center = y_max / 2  # center value used for readability/ease of editing
gimbal_y_pos = -45 # goes from -90 to 0
pix_per_deg = 40 # used to adjust movement sensitivity

def mapObjectPosition(x, y):
	# print object coordinates
	print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))

greenLower = (29, 86, 6) # define lower boundaries of green
greenUpper = (64, 255, 255) # define upper boundaries of green
pts = deque(maxlen=64) # list of tracked points

# load the webcam video stream
vs = VideoStream(usePiCamera=True).start()

# serial connection for pixhawk
connection_string = '/dev/serial/by-id/usb-ArduPilot_fmuv2_1C0034000951353332373834-if00'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True) # Connect to the Vehicle
vehicle.flush() # send a "clear all" type message

# keep looping
while True:
	# grab the current frame
	frame = vs.read()

	# handle the frame from VideoCapture or VideoStream
	frame = frame[1] if args.get("video", False) else frame

	# resize the frame
	frame = imutils.resize(frame, width=600)
	# blur it
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	# convert to HSV color space
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green"
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	# filter false positives
	mask = cv2.erode(mask, None, iterations=2)
	#filter false negatives
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask
		c = max(cnts, key=cv2.contourArea)
		# compute the minimum enclosing circle
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c) # find centroid
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			# draw the centroid
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			# update the list of tracked points
			mapObjectPosition(int(x), int(y))

			x = x_max - x # flip so zero = left
			y = y_max - y # flip so zero = bottom

			# adjust gimbal to follow y centroid
			if (y > (y_center + pix_per_deg)):
				print("above")
				gimbal_y_pos = gimbal_y_pos + (y - y_center)/pix_per_deg
			elif (y < (y_center - pix_per_deg)):
				print("below")
				gimbal_y_pos = gimbal_y_pos - (y_center - y)/pix_per_deg
			if gimbal_y_pos > 0:
				gimbal_y_pos = 0
			elif gimbal_y_pos < -90:
				gimbal_y_pos = -90
			vehicle.gimbal.rotate(int(gimbal_y_pos), 0, 0)

	# update the points queue
	pts.appendleft(center)

	# loop over the set of tracked points
	for i in range(1, len(pts)):
		# ignore non-valid points
		if pts[i - 1] is None or pts[i] is None:
			continue

		# Compute thickness of line
		thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
		# draw the connecting lines
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# Close vehicle object
print("Close vehicle object")
vehicle.close()

# stop the camera video stream
vs.stop()

print("Completed")