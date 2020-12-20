# Object Tracking Drone

Feel free to reach out if you have questions about the project. This project involved designing a drone to perform object tracking with:
- a Pixhawk flight controller
- a RaspberryPi
- a custom servo-based camera gimbal

This project uses the MavLink protocol and OpenCV to allow object detection and tracking on a drone controlled by a pixhawk flight controller.

This project assumes that you already have a XXXX-copter configured using [QGroundControl software](http://qgroundcontrol.com/) which is able to fly using the [stabilized flight mode](http://ardupilot.org/copter/docs/flight-modes.html)

`main_dat.py` is the main program file that contains the mavlink initialization, main logic statements for switching between flight modes and other key componenets of the code. `definitions.py` contains the definitions for OpenCV, the individual mavlink commands etc. The control loop looks like the following:

![The Structure of the program](https://github.com/rlew631/ObjectTrackingDrone/blob/master/ProcessDiagram.jpg?raw=true)

personal notes/reminders:<br/>
go into programming flight modes for the pixhawk...*

## Credits

- [Ardupilot](https://ardupilot.org/)
- [MavLink](https://mavlink.io/en/)
- [PyImageSearch Blog](https://www.pyimagesearch.com/blog/)
- [MJRobot](https://mjrobot.org/)
