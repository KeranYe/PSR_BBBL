# psr_bbbl

## Intro

This is a ROS package for PSR on BeagleBone BLue. Robotic Control Library should be preinstalled on BeagleBone BLue.

In folder `src` are `.cpp` scripts for each node.  In folder `scripts` are `.sh` files for hardware permission.

### How to setup the package

Please install package `psr_msgs` first.

1. Navigate to `~\catkin_ws\src`

2. Copy the download link and `git clone` to current directory

3. GO back to `~\catkin_ws` and use `catkin_make`

## Helpful Linux Commands

`rm -f -r` can remove a non-empty folder clearly.

## Helpful Network Tips

1. In your VM, if `bridge` mode does not work for `ssh` BBBL, then try `NAT` mode and change back to `bridge` mode later.
