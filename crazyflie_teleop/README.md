#crazyflie_teleop
##flyingcar_ros

###Launching Things
`roslaunch crazyflie_teleop start_auto.launch name:=<name> number:=<number> cf_num:=<cf_num>`

Starts the Crazyflie server, connects to mocap, starts RViz, and adds a Crazyflie. Parameters:
	- name: the name of the Crazyflie, settings specified in `flyingcar_ros/crazyflie_driver/config/name.yaml`
	- number: X where X is from Robot_X published by Optitrack associated with the Crazyflie you're connecting to
	- cf_num: the index of the associated simulated crazyflie from the [planner]{https://github.com/braraki/flyingcar_vizplan} 

`roslaunch crazyflie_teleop add_auto.launch  name:=<name> number:=<number> cf_num:=<cf_num>`

Adds another Crazyflie to an already started Server (ie. run to add more Crazyflies **after** running `start_auto.launch`. Parameters same as above. 

----

`roslaunch crazyflie_teleop start_teleop.launch name:=<name>`

Starts server and adds PS3-controlled Crazyflie.

`roslaunch crazyflie_teleop add_teleop.launch name:=<name>`

Adds another Crazyflie with PS3-control. (switch between Crazyflies with start button on controller)

###Scripts

`cf_selector.py` --- Allows switching between multiple Crazyflies in user-controlled teleop mode. 

`crawler_controller.py` --- Controller for crawler module.

`make_rviz.sh` --- Makes RViz from template for specific Crazyflie. 

`ps3_controller.py` --- PS3-controller node. Also allows e-stops in autonomous mode (select button to kill power).

`roller_controller.py` --- Controller for roller module.

`waypoint.py` --- Gets goal points and sends them at appropriate times to wheel and flight controllers.

`wheel_controller.py` --- Controller for wheel module.   


