# ROSpy-Omni-Bot
ROS nodes for controlling a 3WD, Omni-wheeled robot. 
Robot assembly:
- Motor Arduino(Duemilanove clone with motor shield), controls three 12V Faulhaber motors with 64:1 gear ratio
- Encoder Arduino(Mega2560), captures encoder data from motors
- Raspberry Pi 3, two of these control all of the higher level features of the ROS network
- IMU (BNO055), combined with the encoder data provides odometry for localization
- LiDAR lite v3, 2D mapping of the area

Node group includes:
- PID Controller
- Move_to_XY node: takes in a set of Euclidean coordinates then calculates and executes the move
- Linear_travel_PID: uses a heading PID to keep the robot on track
- Rotational PID: uses a pulse/second PID for each wheel to maintain on the spot rotation.

 
