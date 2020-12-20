#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)

This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.

Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
"""  # proprietary message from dronekit devs

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

# global variable(s)
object_x = 0  # initializing x variable
object_y = 0  # initializing y variable

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


# print object coordinates
def object_tracking(x, y):
    print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))
    global object_x  # importing the global object_x variable
    global object_y  # importing the global object_y variable
    x_error = x -    # writing the global object_x variable
    object_x = y     # writing the global object_y variable
    condition_yaw(heading, relative=False)

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

    if not args.get("video", False):  # if a video path was not supplied, grab the reference to the webcam
        vs = VideoStream(src=0).start()
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


# <editor-fold desc="Definitions and code for dronekit">
def arm_and_takeoff(tgt_altitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(tgt_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= tgt_altitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(1)  # Arm and take of to altitude of 1 meter

"""
The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
* MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed (latitude, longitude, altitude)
* MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.

The full set of available commands are listed here:
http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
"""  # Convenience functions for sending immediate/guided mode commands to control the Copter


def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s, NOT SUPPORTED
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def set_roi(location):  # points gimbal to region of interest (ROI), global location
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see: 
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # command
        0,  # confirmation
        0, 0, 0, 0,  # params 1-4
        location.lat,
        location.lon,
        location.alt
    )
    # send command to vehicle
    vehicle.send_mavlink(msg)


"""
In particular these
make it easy to navigate in terms of "metres from the current position" when using commands that take 
absolute positions in decimal degrees.

The methods are approximations only, and may be less accurate over longer distances, and when close 
to the Earth's poles.

Specifically, it provides:
* get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
* get_distance_metres - Get the distance between two LocationGlobal objects in metres
* get_bearing - Get the bearing in degrees to a LocationGlobal
"""  # Functions to make it easy to convert between the different frames-of-reference


"""
The methods include:
* goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
* goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
    MAV_FRAME_BODY_NED frame
* goto - A convenience function that can use Vehicle.simple_goto (default) or 
    goto_position_target_global_int to travel to a specific position in metres 
    North and East from the current location. 
    This method reports distance to the destination.
"""  # Move vehicle to a specified position (as opposed to controlling movement by setting velocity components)


def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111111000,  # type_mask (only speeds enabled)
        aLocation.lat * 1e7,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon * 1e7,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt,
        # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0,  # X velocity in NED frame in m/s
        0,  # Y velocity in NED frame in m/s
        0,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto(d_north, d_east, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """

    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, d_north, d_east)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    # print "DEBUG: targetLocation: %s" % targetLocation
    # print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
        # print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance <= targetDistance * 0.01:  # Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)


"""
The two functions use different MAVLink commands. The main difference is
that depending on the frame used, the NED velocity can be relative to the vehicle
orientation.

The methods include:
* send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
* send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
"""  # Functions that move the vehicle by specifying the velocity components in each direction


def send_ned_velocity(vel_x, vel_y, vel_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vel_x, vel_y, vel_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def send_global_velocity(vel_x, vel_y, vel_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        vel_x,  # X velocity in NED frame in m/s
        vel_y,  # Y velocity in NED frame in m/s
        vel_z,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


"""
The method is called indirectly via a custom "goto" that allows the target position to be
specified as a distance in metres (North/East) from the current position, and which reports
the distance-to-target.
"""  # Fly a triangular path using the standard Vehicle.simple_goto() method
# <editor-fold desc="Triangle path, Vehicle.simple_goto()">
'''
print("TRIANGLE path using standard Vehicle.simple_goto()")

print("Set ground speed to 5m/s.")
vehicle.groundspeed = 5

print("Position North 80 West 50")
goto(80, -50)

print("Position North 0 East 100")
goto(0, 100)

print("Position South 80 West 50")
goto(-80, -50)
'''
# </editor-fold>


"""
The command is called from goto_position_target_global_int() (via `goto`).

The goto_position_target_global_int method is called indirectly from a custom "goto" that allows 
the target position to be specified as a distance in metres (North/East) from the current position, 
and which reports the distance-to-target.

The code also sets the speed (MAV_CMD_DO_CHANGE_SPEED). In AC3.2.1 Copter will accelerate to this speed 
near the centre of its journey and then decelerate as it reaches the target. 
In AC3.3 the speed changes immediately.
"""  # Fly triangular path using SET_POSITION_TARGET_GLOBAL_INT and target position (rather than using velocity vectors)
# <editor-fold desc="Triangle path, SET_POSITION_TARGET_GLOBAL_INT without velocity parameters">
'''
print("TRIANGLE path using standard SET_POSITION_TARGET_GLOBAL_INT message and with varying speed.")
print("Position South 100 West 130")

print("Set ground speed to 5m/s.")
vehicle.groundspeed = 5
goto(-100, -130, goto_position_target_global_int)

print("Set ground speed to 15m/s (max).")
vehicle.groundspeed = 15
print("Position South 0 East 260")
goto(0, 260, goto_position_target_global_int)

print("Set airspeed to 10m/s (max).")
vehicle.airspeed = 10

print("Position North 100 West 130")
goto(100, -130, goto_position_target_global_int)
'''
# </editor-fold>


"""
The command is called from goto_position_target_local_ned() (via `goto`).

The position is specified in terms of the NED (North East Down) relative to the Home location.

WARNING: The "D" in NED means "Down". Using a positive D value will drive the vehicle into the ground!

The code sleeps for a time (DURATION) to give the vehicle time to reach each position (rather than 
sending commands based on proximity).

The code also sets the region of interest (MAV_CMD_DO_SET_ROI) via the `set_roi()` method. This points the 
camera gimbal at the the selected location (in this case it aligns the whole vehicle to point at the ROI).
"""  # Fly square path using SET_POSITION_TARGET_LOCAL_NED and target position (rather than using velocity vectors)
# <editor-fold desc="Square path, SET_POSITION_TARGET_LOCAL_NED without velocity parameters">
'''
print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
DURATION = 20  # Set duration for each segment.

print("North 50m, East 0m, 10m altitude for %s seconds" % DURATION)
goto_position_target_local_ned(50, 0, -10)
print("Point ROI at current location (home position)")
# NOTE that this has to be called after the goto command as first "move" command of a particular type
# "resets" ROI/YAW commands
set_roi(vehicle.location.global_relative_frame)
time.sleep(DURATION)

print("North 50m, East 50m, 10m altitude")
goto_position_target_local_ned(50, 50, -10)
time.sleep(DURATION)

print("Point ROI at current location")
set_roi(vehicle.location.global_relative_frame)

print("North 0m, East 50m, 10m altitude")
goto_position_target_local_ned(0, 50, -10)
time.sleep(DURATION)

print("North 0m, East 0m, 10m altitude")
goto_position_target_local_ned(0, 0, -10)
time.sleep(DURATION)
'''
# </editor-fold>


"""
The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

The code also sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method in each segment
so that the front of the vehicle points in the direction of travel
"""  # Fly square path using velocity vectors (SET_POSITION_TARGET_LOCAL_NED command with velocity parameters enabled)
# <editor-fold desc="Square path, SET_POSITION_TARGET_LOCAL_NED with velocity parameters">
'''
# Set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
NORTH = 2
SOUTH = -2

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
EAST = 2
WEST = -2

# Note for vz: 
# vz < 0 => ascend
# vz > 0 => descend
UP = -0.5
DOWN = 0.5

# Square path using velocity
print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and velocity parameters")

print("Yaw 180 absolute (South)")
condition_yaw(180)

print("Velocity South & up")
send_ned_velocity(SOUTH, 0, UP, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 270 absolute (West)")
condition_yaw(270)

print("Velocity West & down")
send_ned_velocity(0, WEST, DOWN, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 0 absolute (North)")
condition_yaw(0)

print("Velocity North")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 90 absolute (East)")
condition_yaw(90)

print("Velocity East")
send_ned_velocity(0, EAST, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)
'''
# </editor-fold>


"""
The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

The code sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method using relative headings
so that the front of the vehicle points in the direction of travel.

At the end of the second segment the code sets a new home location to the current point.
"""  # Fly DIAMOND path using velocity vectors (SET_POSITION_TARGET_GLOBAL_INT command with velocity parameters enabled)
# <editor-fold desc="Diamond path, SET_POSITION_TARGET_GLOBAL_INT with velocity parameters">
'''
print("DIAMOND path using SET_POSITION_TARGET_GLOBAL_INT and velocity parameters")
# vx, vy are parallel to North and East (independent of the vehicle orientation)

print("Yaw 225 absolute")
condition_yaw(225)

print("Velocity South, West and Up")
send_global_velocity(SOUTH, WEST, UP, DURATION)
send_global_velocity(0, 0, 0, 1)

print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

print("Velocity North, West and Down")
send_global_velocity(NORTH, WEST, DOWN, DURATION)
send_global_velocity(0, 0, 0, 1)

print("Set new home location to current location")
vehicle.home_location = vehicle.location.global_frame
print("Get new home location")
# This reloads the home location in DroneKit and GCSs
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
print(" Home Location: %s" % vehicle.home_location)

print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

print("Velocity North and East")
send_global_velocity(NORTH, EAST, 0, DURATION)
send_global_velocity(0, 0, 0, 1)

print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

print("Velocity South and East")
send_global_velocity(SOUTH, EAST, 0, DURATION)
send_global_velocity(0, 0, 0, 1)
'''
# </editor-fold>


"""
The example is completing. LAND at current location.
"""

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
# </editor-fold>
