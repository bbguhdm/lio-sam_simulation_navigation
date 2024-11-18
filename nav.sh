
cd /home/hdm/lio-sam_simulation_navigation
source install/setup.bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh

# 命令数组  . install/setup.bash
commands=(
"gnome-terminal -- bash -c ' ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch pointcloud_downsampling pointcloud_downsampling.launch.py  ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch lio_sam run.launch.py   ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch robot_navigation2 navigation2.launch.py ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch robot_bringup bringup.launch.py    ; exec bash;'"
)


cd /home/hdm/lio-sam_simulation_navigation
for command in "${commands[@]}"; do
  
  eval "$command"
done






                                                                                                                  





