#!/bin/bash

name=$1

cd $(dirname $0)
cd ../launch

rm -f "launch/crazyflie_pos.rviz"

sed "s/ROBOT_NAME/$name/g" "pos_template.rviz" >> "crazyflie_pos.rviz"

exit 0