# <editor-fold desc="different ways of setting up flight paths">
"""
The command is called from goto_position_target_local_ned() (via `goto`).

The position is specified in terms of the NED (North East Down) relative to the Home location.

WARNING: The "D" in NED means "Down". Using a positive D value will drive the vehicle into the ground!

The code sleeps for a time (DURATION) to give the vehicle time to reach each position (rather than 
sending commands based on proximity).

The code also sets the region of interest (MAV_CMD_DO_SET_ROI) via the `set_roi()` method. This points the 
camera gimbal at the the selected location (in this case it aligns the whole vehicle to point at the ROI).
"""  # Fly square path using SET_POSITION_TARGET_LOCAL_NED and target position (rather than using velocity vectors)
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

"""
The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

The code also sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method in each segment
so that the front of the vehicle points in the direction of travel
"""  # Fly square path using velocity vectors (SET_POSITION_TARGET_LOCAL_NED command with velocity parameters enabled)
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

"""
The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

The code sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method using relative headings
so that the front of the vehicle points in the direction of travel.

At the end of the second segment the code sets a new home location to the current point.
"""  # Fly DIAMOND path using velocity vectors (SET_POSITION_TARGET_GLOBAL_INT command with velocity parameters enabled)
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