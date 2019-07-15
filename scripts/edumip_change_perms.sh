#!/bin/sh
# 2017-11-29 LLW shell script for changing ownership and sticky bit for edumip_balance_ros
# usage: ~/bin/edumip_change_perms.sh
#
echo ls -l ~/catkin_ws/devel/lib/psr_bbbl/psr_bbbl_drive
ls -l ~/catkin_ws/devel/lib/psr_bbbl/psr_bbbl_drive

echo sudo chown root:root  ~/catkin_ws/devel/lib/psr_bbbl/psr_bbbl_drive
sudo chown root:root  ~/catkin_ws/devel/lib/psr_bbbl/psr_bbbl_drive

echo sudo chmod u+s  ~/catkin_ws/devel/lib/psr_bbbl/psr_bbbl_drive
sudo chmod u+s  ~/catkin_ws/devel/lib/psr_bbbl/psr_bbbl_drive

echo ls -l ~/catkin_ws/devel/lib/psr_bbbl/psr_bbbl_drive
ls -l ~/catkin_ws/devel/lib/psr_bbbl/psr_bbbl_drive
