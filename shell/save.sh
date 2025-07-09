#!/bin/bash

source /opt/ros/humble/setup.bash
 
 
 
 echo "save"
 ros2 service call /map_save std_srvs/srv/Trigger


