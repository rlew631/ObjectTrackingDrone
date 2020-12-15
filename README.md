# Object Tracking Drone

NOTE: for the time being all of this is for my own personal reference while putting together a properly organized project. feel free to reach out if you have questions abotu the project.

**Drone w/ camera gimbal controlled by Pixhawk and Raspberry Pi running OpenCV**

The goal of this project is to use the MavLink protocol and OpenCV to allow object detection and tracking on a drone controlled by a pixhawk flight controller.

This project assumes that you already have a XXXX-copter configured using [QGroundControl software](http://qgroundcontrol.com/) which is able to fly using the [stabilized flight mode](http://ardupilot.org/copter/docs/flight-modes.html)

`main_dat.py` is the main program file that contains the mavlink initialization, main logic statements for switching between flight modes and other key componenets of the code. `definitions.py` contains the definitions for OpenCV, the individual mavlink commands etc. The control loop looks like the following:

![The Structure of the program](https://github.com/rlew631/ObjectTrackingDrone/blob/master/ProcessDiagram.jpg?raw=true)

personal notes/reminders:<br/>
*put in thanks to: mavlink community pyimagesearch and mjrovai for tutorials on object tracking and rpi io stuff<br/>
go into programming flight modes for the pixhawk...*
