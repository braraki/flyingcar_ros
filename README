ROS PACKAGE FOR CSAIL FLYINGCAR PROJECT
B. Araki, S. Pohorecky, C. Qiu, J. Strang

To run the crazyflie with PS3 joystick control:
```roslaunch crazyflie_teleop start_teleop.launch name:=<name>```

To add another flie to the teleop once running:
```roslaunch crazyflie_teleop teleop_ps3.launch name:=<name>```

Crazyflie YAML config files are contained in `crazyflie_driver/config`

To use Optitrack cameras for teleop, the mocap_optitrack and the robot_localization packages must be installed. To hover at a predefined location, run:
```roslaunch crazyflie_teleop hover_mocap.launch name:=<name> x:=<x> y:=<y> z:=<z>```

Most of the crazyflie code modified from the crazyflie_ros package by W. Hoenig. Thanks!
