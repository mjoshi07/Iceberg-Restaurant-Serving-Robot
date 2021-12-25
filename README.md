# Iceberg-Restaurant-Serving-Robot
Mobile manipulator which can autonomously navigate in a restaurant environment while serving food to the customers

# Robot picking up an object and navigating to the table 
![image](https://github.com/mjoshi07/Iceberg-Restaurant-Serving-Robot/blob/main/data/iceberg_small.gif)


## Dependencies
* MoveIt
```
sudo apt-get install ros-noetic-moveit
```
* Gazebo link attacher
* To run the kinematics scripts (python3)
```
pip3 install sympy
```

## Build instructions
* [RECOMMENDED]Create a new workspace named "iceberg_ws" and build it, then copy or merge the src/ folder inside this workspace
* Compile the workspace
```
catkin_make
```
* From workspace root folder, run the following command
```
source devel/setup.bash
```

## Run instructions
* Launch the Iceberg robot into the gazebo environment with following command
```
roslaunch mobile_manipulator_body mobile_manipulator_gazebo.launch
```
* Open a new terminal for running the moveit backend for Iceberg and execute these two commands
```
source devel/setup.bash
```
```
roslaunch iceberg_moveit_config demo.launch
```
* To operate Iceberg in this environment open a new terminal, and execute following command
```
source devel/setup.bash
```
```
rosrun mobile_manipulator_body iceberg.py
```
