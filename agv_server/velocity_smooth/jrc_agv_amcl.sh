#!/bin/sh
roslaunch agv_tcp_velocity laser.launch&
sleep 3s
roslaunch ira_laser_tools laserscan_multi_merger.launch&
sleep 2s
roslaunch laser_scan_matcher demo.launch&
sleep 1s
rosrun map_server map_server /home/agv/jd_ws/src/agv/map/smooth_test_4.yaml&
sleep 2s
roslaunch amcl amcl_omni.launch&
sleep 2s
rosrun rviz rviz&
sleep 2s
rosrun velocity_smooth velocity_smooth&
sleep 2s

