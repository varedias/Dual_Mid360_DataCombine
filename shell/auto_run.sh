#!/bin/bash 

source /opt/ros/humble/setup.bash
source /home/elf/Downloads/ws_livox/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/elf/Downloads/Livox-SDK2/build/sdk_core



echo "IP set"
sudo ip addr flush dev wlP4p65s0
sudo ip addr add 192.168.1.50/24 dev wlP4p65s0
sudo ip route add default via 192.168.1.1 

echo "start msg Please make sure your lidar is opened"
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 5

echo "start merge data"
cd /home/elf/Downloads/MultisenseCombine/build
./merge_cloud_node & 
sleep 3

echo "wait for node"
while ! ros2 topic list | grep -q "/merged_cloud"; do 
sleep 1 
done 

echo " start fast lio please don not  move the device! " 
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml & 
sleep 3


