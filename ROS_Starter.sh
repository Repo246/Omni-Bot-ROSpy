#
#
#!/bin/bash

# Start roscore first 
roscore &

sleep 5

# Start IMU node
#python /home/ubuntu/Desktop/Project_Files/ROS_System_Control/BNO055_node/bno055_node.py &

sleep 7

# Start encoders_working.py
python ~/Desktop/Project_Files/ROS_System_Control/Encoder_node/encoders.py &

sleep 2

# Start the Extended Kalman filter
roslaunch ~/Desktop/Project_Files/ROS_System_Control/robot-localization/robot_localization.launch &

sleep 2

# Start move_to_xy_serial.py
python ~/Desktop/Project_Files/ROS_System_Control/Move_to_xy/move_to_xy.py &

