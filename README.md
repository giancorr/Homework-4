##      📔 HOMEWORK 4
This repository has been created in order to fulfill the fourth homework of RoboticsLab 2024/2025 class. 

## 💻 Usage 
###      🔨 Build
First of all, clone this repository in your ros2_ws/src folder
```
git clone https://github.com/PasFar/Homework-4.git
```
Then, build the packages using the following command inside the ros2_ws folder and source the setup.bash file 
```
colcon build
. install/setup.bash
```
### ✅ How to run
You have different option for running the nodes.

To test the robot's feature of following the assigned waypoints
   ```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
   ```
This will spawn the mobile robot inside the race field on gazebo in the assigned pose [-3.0, 3.5, 0.100] with a yaw angle of 1.57 rad with respect to the map frame.
Then, open another terminal, connect to the same docker container and run:
   ```
   . install/setup.bash
   ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
   ```
This will start nav2 node and slam node. Then in another terminal:
   ```
    . install/setup.bash
    ros2 launch rl_fra2mo_description display_fra2mo.launch.py use_explore:=true
   ```
This will open rviz with the configuration "explore.rviz".
Finally, open another terminal, connect to the same docker container and run:
   ```
   . install/setup.bash
   ros2 run rl_fra2mo_description follow_waypoints waypoints
   ```
The robot will move to the assigned waypoint described in the config/waypoints.yaml file with the order (3)->(4)->(2)->(1).
Trajectories results can be seen in the related report.

In order to test the mapping feature the commands are the same except for the last one which is: 
   ```
   ros2 run rl_fra2mo_description follow_waypoints mapping
   ```
The manipulator will move and map his sourroundings. The map results can be seen in the related report. 
In this case the given waypoints will be the ones in config/mapping.yaml.

If you want to test the aruco detection you can run:
   ```
   colcon build
   . install/setup.bash
   ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
   ```
Then you need to run in a second terminal: 
   ```
   . install/setup.bash
   ros2 launch rl_fra2mo_description aruco_fra2mo.launch.py 
   ```
This launch file will start: explore node, aruco detection node and aruco_tf node which will retrieve the aruco position with respect to map frame and pusblishes on the topic "/aruco_pose_in_map" and, finally, pusblishes the static tf of the marker.

In another terminal: 
   ```
   . install/setup.bash
   ros2 launch rl_fra2mo_description display_fra2mo.launch.py use_slam:=true
   ```
In this case the rviz config will be "aruco_view.rviz" which will open slam view with addition of tf visualization and also the image view plugin with the topic /aruco_single/result.

In a last terminal: 
   ```
   . install/setup.bash
   ros2 run rl_fra2mo_description follow_waypoints.py aruco
   ```
If you want to also check the detected aruco pose with respect to the map frame you can run in another terminal: 
   ```
   . install/setup.bash
   ros2 topic echo /aruco_pose_in_map
   ```
The robot will reach a position close to the aruco marker, detect it and then head back to the starting position.
Results and documentation will be found in the attached report. 
