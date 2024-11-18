# lio-sam_simulation_navigation

#### 介绍
本项目在仿真环境中使用lio_sam算法进行定位，从而使用navigation进行机器人导航。

注：本项目使用的lio_sam算法对原版的做了略微的修改，以适配仿真环境。原版lio_sam算法链接：https://github.com/TixiaoShan/LIO-SAM/tree/ros2

支持版本：Ubuntu22.04 ROS2-humble

![f19afb5d_14318961](https://github.com/user-attachments/assets/b55d09f4-266b-4718-a607-1bb0cfc348a2)


使用说明

1. 克隆

git clone https://gitee.com/dedm/lio-sam_simulation_navigation.git

2. 编译

colcon build 

3. 运行

./nav.sh
