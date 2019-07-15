#!/bin/sh
# 2017-11-29 LLW shell script for changing ownership and sticky bit for edumip_balance_ros
# usage: ~/bin/edumip_change_perms.sh
#
echo ls -l ~/catkin_ws/devel/lib/psr/psr_drive_dutycycle
ls -l ~/catkin_ws/devel/lib/psr/psr_drive_dutycycle

echo sudo chown root:root  ~/catkin_ws/devel/lib/psr/psr_drive_dutycycle
sudo chown root:root  ~/catkin_ws/devel/lib/psr/psr_drive_dutycycle

echo sudo chmod u+s  ~/catkin_ws/devel/lib/psr/psr_drive_dutycycle
sudo chmod u+s  ~/catkin_ws/devel/lib/psr/psr_drive_dutycycle

echo ls -l ~/catkin_ws/devel/lib/psr/psr_drive_dutycycle
ls -l ~/catkin_ws/devel/lib/psr/psr_drive_dutycycle
