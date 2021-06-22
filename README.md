

Launch ursim
Ensure program is just external control w ip 10.0.2.15


start terminal ctrl+alt+t
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=10.0.2.15
```

play program on robot

new terminal tab ctrl+shift+t
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch
```

new terminal tab ctrl+shift+t
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```


