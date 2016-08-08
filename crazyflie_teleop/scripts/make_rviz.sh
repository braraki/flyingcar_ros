#!/bin/bash

cd $(dirname $0)
cd ../launch

rm -f "crazyflie_pos.rviz"

# cat ./rviz_template/rviz_head >> "crazyflie_pos.rviz"
# echo -e "\n" >> "crazyflie_pos.rviz"

# while [ $# -ne 0 ]; do
# 	sed "s/ROBOT_NAME/$1/g" ./rviz_template/rviz_robot_content >> "crazyflie_pos.rviz"
# 	echo -e "\n" >> "crazyflie_pos.rviz"
# 	shift
# done

# cat ./rviz_template/rviz_foot >> "crazyflie_pos.rviz"

sed "s/ROBOT_NAME/$1/g" pos_template.rviz >> "crazyflie_pos.rviz"

exit 0