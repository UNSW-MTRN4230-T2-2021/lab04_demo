# Lab04 Robot Control Demonstration

## Start up the robot simulator 

Launch URSim

Ensure the program has just the external control with the IP address set to `10.0.2.15`


NB: if you have overwritten the program, you can find the external control under URCaps

![image](https://user-images.githubusercontent.com/29705680/123183147-3f97cb80-d4d4-11eb-8929-9c10c71c5bc5.png)


## Start up the communication between ROS and URSim
Open a terminal with `ctrl+alt+t`
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=10.0.2.15
```

Once the terminal has finished printing the startup output, play the program on URSim.

The terminal should output logging information saying the robot is ready to recieve commands
```
[ INFO] [1624490993.175980451]: Robot requested program
[ INFO] [1624490993.176238507]: Sent program to robot
[ INFO] [1624490993.243437722]: Robot ready to receive control commands.
```

## Start up the Path Planning node 
Open a new terminal tab with `ctrl+shift+t`
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch
```
## Start up the GUI for moving the robot
Open a new terminal tab with `ctrl+shift+t`
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```

## Make the robot move
Click and hold the TCP of the robot and drag it to a new location.

Select `Plan` from the MotionPlanning Commands section of the GUI.

If the robot was able to plan a path, the execute button will be clickable.

Select `Execute` from the MotionPlanning Commands section of the GUI.

Watch the UR5e perform the motion in URSim as the robot performs the motion in the RViz window.

# Lab Task setup

For setting up for the lab task:
```bash
cd # Ensure we are in the home directory
mkdir ~/lab04_repo/ # Create directory for repository
mkdir -p ~/lab04_ws/src # Create directory for ROS workspace and its source subdirectory 
cd lab04_ws # Change into Lab04 ROS Workspace
catkin_make # Initialise Workspace
source devel/setup.bash # Tell ROS what our workspace looks like
cd src && catkin_create_pkg lab04_twist_controller std_msgs rospy roscpp geometry_msgs # Create ROS package
mv lab04_twist_controller ~/lab04_repo/ # Move the package into our repository for tracking
ln -s ~/lab04_repo/lab04_twist_controller ~/lab04_ws/src/ # Symbolically link the package into our ROS workspace so that the workspace can still see it
cd ~/lab04_ws && catkin_make # Move into the root of the ROS workspace and compile and build the workspace
cd ~/lab04_repo && git init && git add . # Move into repository, initialise repo and add package to repo
code ./lab04_twist_controller/
```
Make a private repository on Github and follow the instructions to add an existing repo to GitHub.

If you want to avoid using your username and password everytime ou push and pull, follow GitHubs guide on setting up SSH keys https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh and ensure you use the ssh url not the https url for linking the repo on the vm to your remote GitHub repo.

After that, you're all set up! You can start programming your ROS nodes!
