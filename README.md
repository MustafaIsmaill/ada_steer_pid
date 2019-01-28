# ADA PID Control

ADA_PID_Control is **ROS** package written in Python which calculates steering angle for lane keeping control using PID controller. PID package was developed by Andy Zelenak. ADA_PID_control is PID ROS package but with the addition of two nodes for calculating state of vehicle which is lateral position error using path points and GPS raw data (UTMX , UTMY) and the set point desired. The controller node is the main node in the package. It implements the PID algorithm and applies control effort to try and make plant state equal setpoint.
please see the documentation here: [http://wiki.ros.org/pid](http://wiki.ros.org/pid).


## 1. Usage


1. run road_processing_planning package
2. run `$ roslaunch pid servo_sim.launch ` to launch ADA_steer_pid package
3. the maximum and the minimum steering angle can be set from the launch file change the value of upper and the lower limit

## 2. Topics

### 2.1 Subscribed topics
    1./ada/fix (sensor_msgs/NavSatFix.msg) to get the GPS raw data.

    2. /control_effort (std_msgs/Float64.msg) to get the Control effort which is the steering angle

### 2.2 Published topics
    1. /ackermann_cmd (ackermann_msgs/AckermannDrive.msg) where the steering angles and the steering angle rate is published 

    2. /set_point (std_msgs/Float64.msg) where the desired set point is published

    3. /state (std_msgs/Float64.msg) where the lateral position error is published

