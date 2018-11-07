#!/bin/sh
roslaunch agv_tcp_velocity laser.launch&
sleep 3s
roslaunch ira_laser_tools laserscan_multi_merger.launch&
sleep 2s
roslaunch laser_scan_matcher demo.launch&
sleep 1s
rosrun map_server map_server /home/brucechen/nav_ws/src/map/smooth_test_3.yaml&
sleep 2s
roslaunch amcl amcl_omni.launch&
sleep 2s
rosrun rviz rviz&
sleep 2s
rosrun velocity_smooth velocity_smooth&
sleep 2s
sudo chmod 666 /dev/ttyUSB0&
sleep 1s
rosrun soccer_maxon soccer_maxon_node&
sleep 2s
