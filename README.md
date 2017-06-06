# ROSpy-Omni-Bot
ROS nodes for controlling a 3WD, Omni-wheeled robot. 
Robot assembly:
- Motor Arduino(Duemilanove clone with motor shield), controls three 12V Faulhaber motors with 64:1 gear ratio
- Encoder Arduino(Mega2560), captures encoder data from motors
- Raspberry Pi 3, two of these control all of the higher level features of the ROS network
- IMU (BNO055), combined with the encoder data provides odometry for localization
- LiDAR lite v3, 2D mapping of the area

ROS Node group included:
- Move_to_XY node: takes in a set of Euclidean coordinates then calculates and executes the move
- Encoder node: Retrieves data from the Encoder Arduino, calculates a position and publishes this to the ROS network
- Jostick Control node: Allows a joystick(we used Logitech Rumblepad cordless) to control the omnibot

Arduino:
- Linear_travel_PID: uses a heading PID to keep the robot on track
- Rotational PID: uses a pulse/second PID for each wheel to maintain on the spot rotation.

Libraries:
- PID Controller library
- Wheel Library (Adapted from Nexus Robots MotorWheel header files)

ROS Node group NOT included, but comparable is required:
- IMU node setup to send data to a topic subscribed to by move_to_xy.py
  https://github.com/mdrwiega/bosch_imu_driver
- robot_localization node: used to filter odometry and LiDAR data and provide accurate positioning 
  http://wiki.ros.org/robot_localization or
  http://wiki.ros.org/robot_pose_ekf, etc.



 
