mx_dynamixel
===============

ROS stack for interfacing with Robotis Dynamixel MX series protocol version 2.0.
Note that the protocol 1.0 firmware is incompatible with this stack and explicitly assumes the connected motors have the upgraded firmwarefrom Robotis.

Currently this package extends the functionality of the dynamixel_motor stack, using 2.0 communication.
If your motors are using firmware that uses protocol 1.0 communication, use the dynamixel_motor stack instead.
