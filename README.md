

Launch ursim

Ensure program has just the external control with the ip address set to 10.0.2.15

NB: if you have overwritten the program, you can find the external control under URCaps


Start a terminal with `ctrl+alt+t`
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=10.0.2.15
```

Play the program on URSim

Open a new terminal tab with `ctrl+shift+t`
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch
```

Open a new terminal tab `ctrl+shift+t`
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```

For setting up for the lab task:
```bash
cd
mkdir -p ~/lab04_ws/src
mkdir -p ~/lab04_repo/
cd lab04_ws
catkin_make
source devel/setup.bash
catkin_create_pkg lab04_Pose_Controller std_msgs rospy roscpp geometry_msgs
mv lab04_Pose_Controller ~/lab04_repo/
ln -s ~/lab04_repo/lab04_Pose_Controller ~/lab04_ws/src/
cd ~/lab04_ws
catkin_make
```
Start programming!
