#!/bin/sh
# 2017-11-29 LLW shell script for changing ownership and sticky bit for edumip_balance_ros
# usage: ~/bin/edumip_change_perms.sh
#
echo ls -l ~/catkin_ws/devel/lib/PSR_BBBL/psr_drive_encoder
ls -l ~/catkin_ws/devel/lib/PSR_BBBL/psr_drive_encoder

echo sudo chown root:root  ~/catkin_ws/devel/lib/PSR_BBBL/psr_drive_encoder
sudo chown root:root  ~/catkin_ws/devel/lib/PSR_BBBL/psr_drive_encoder

echo sudo chmod u+s  ~/catkin_ws/devel/lib/PSR_BBBL/psr_drive_encoder
sudo chmod u+s  ~/catkin_ws/devel/lib/PSR_BBBL/psr_drive_encoder

echo ls -l ~/catkin_ws/devel/lib/PSR_BBBL/psr_drive_encoder
ls -l ~/catkin_ws/devel/lib/PSR_BBBL/psr_drive_encoder
