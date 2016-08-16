#crazyflie_tools
##flyingcar_ros

###Scripts
`path_display.py` --- displays Crazyflie path-of-travel in RViz

`pose_converter.py` --- converts `/Robot_X/pose` from mocap node into PoseWithCovarianceStamped on topic `<name>/pose_mocap` **and** converts localized pose `<name>/pose_localization` to PoseStamped on topic `<name>/pose`

`sim_path_display.py` --- displays path from the [planer]{http://github.com/braraki/flyingcar_vizplan} and current goal of Crazyflie  