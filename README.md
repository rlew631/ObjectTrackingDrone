# Object_Tracking_Drone

NOTE: for the time being all of this is for my own personal reference while putting together a properly organized project. feel free to reach out if you have questions abotu the project.

**Drone w/ camera gimbal controlled by Pixhawk and Raspberry Pi running OpenCV**

The goal of this project is to use the MavLnk protocol and OpenCV to allow object detection and tracking on a drone controlled by a pixhawk flight controller.

This project assumes that you already have a XXXX-copter configured using [QGroundControl software](http://qgroundcontrol.com/) which is able to fly using the [stabilized flight mode](http://ardupilot.org/copter/docs/flight-modes.html)

Currently the main_dat.py file is the main program file that contains the mavlink initialization, main logic statements for switching between flight modes and other key componenets of the code. The definitions.py file contains the definitions for OpenCV, the individual mavlink commands etc.

personal notes/reminders:<br/>
*put in thanks to: mavlink community pyimagesearch and mjrovai for tutorials on object tracking and rpi io stuff<br/>
mention using python -m pip for installing files through python2.7, 3.8 etc.<br/>
go into programming flight modes for the pixhawk...*
