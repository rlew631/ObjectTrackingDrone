#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This file contains the definitions to interface with:
    Dronekit to interface with the pixhawk and gimbal
    OpenCV to perform object detection
    Perform
'''


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


# Definitions for dronekit"
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


def condition_gimbal_pitch(deg):
    # from: https://diydrones.com/profiles/blogs/idiot-s-guide-to-dronekit-python-a-journey-to-whoz-chillin
    # create the CONDITION_GIMBAL_PITCH command using mount_control_encode()
    msg = vehicle.message_factory.mount_control_encode(
        0, 1,  # target system, target component
        deg * 100,  # pitch is in centidegrees
        0.0,  # roll
        0,  # yaw is in centidegrees
        0)  # save position
    vehicle.send_mavlink(msg) # send command to vehicle


def goto_position_target_local_ned(north, east, down):
    """
    Local navigation without GPS:
        Sets position using SET_POSITION_TARGET_LOCAL_NED command in MAV_FRAME_BODY_NED frame

    WARNING: The "D" in NED means "Down". Using a positive D value will drive the vehicle into the ground!
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
    Local navigation with GPS:
    A convenience function that can use Vehicle.simple_goto (default) or goto_position_target_global_int to
    travel to a specific position in metres

    North and East from the current location.
    This method reports distance to the destination.
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