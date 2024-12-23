#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import yaml
import os
import math
import sys


def rpy_to_quaternion(roll, pitch, yaw):

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return {
        'x': qx,
        'y': qy,
        'z': qz,
        'w': qw
    }


def load_waypoints(package_path):
    
    if sys.argv[1] == 'mapping':
        config_path = '/home/user/ros2_ws/src/rl_fra2mo_description/config/mapping.yaml'
    elif sys.argv[1] == 'waypoints':
        config_path = '/home/user/ros2_ws/src/rl_fra2mo_description/config/waypoints.yaml'
    elif sys.argv[1] == 'aruco':
        config_path = '/home/user/ros2_ws/src/rl_fra2mo_description/config/aruco_waypoint.yaml'
    else:
        print("Action not well-specified")
        return

    try:
        with open(config_path, 'r') as file:
            waypoints_data = yaml.safe_load(file)
        return waypoints_data.get('waypoints', [])
    except FileNotFoundError:
        print(f"Errore: File {config_path} non trovato!")
        return []
    except Exception as e:
        print(f"Errore durante il caricamento dei waypoint: {e}")
        return []




def main():
    rclpy.init()

    node = rclpy.create_node('follow_waypoints')

    

    if sys.argv[1] == 'aruco':
            node.aruco_detected = False  # Flag for ArUco detection
            def aruco_pose_callback(msg):
                # ArUco callback 
                if not node.aruco_detected:
                    node.get_logger().info("ArUco marker detected!")
                    node.aruco_detected = True

            aruco_pose_sub = node.create_subscription(
                PoseStamped,
                '/aruco_single/pose',
                aruco_pose_callback,
                10
            )
        
    
    navigator = BasicNavigator()

    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    waypoints = load_waypoints(package_path)

    if not waypoints:
        print("Nessun waypoint caricato. Uscita.")
        return

    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        
        #Set the position after the transformation
        pose.pose.position.x = -(transform["position"]["y"]-3.5)
        pose.pose.position.y = (transform["position"]["x"]+3)
        pose.pose.position.z = transform["position"]["z"]
        
        #Set the quaternion after the transformation
        rpy = transform.get("orientation", {})
        roll = rpy.get("roll", 0)
        pitch = rpy.get("pitch", 0)
        yaw = rpy.get("yaw", 0)+1.57
        
        quaternion = rpy_to_quaternion(roll, pitch, yaw)

        pose.pose.orientation.x = quaternion['x']
        pose.pose.orientation.y = quaternion['y']
        pose.pose.orientation.z = quaternion['z']
        pose.pose.orientation.w = quaternion['w']
        
        return pose

    if sys.argv[1] == 'mapping':
        waypoint_order = [0, 1, 2, 3, 4, 5]
    elif sys.argv[1] == 'waypoints':
        waypoint_order = [2, 3, 1, 0]
    elif sys.argv[1] == 'aruco':
        waypoint_order = [0, 1]
    else:
        print("Action not well-specified")
        return

    
    goal_poses = [create_pose(waypoints[i]) for i in waypoint_order]

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    nav_start = navigator.get_clock().now()

    #Waypoint loop
    for i, goal_pose in enumerate(goal_poses):
        print(f'Navigating to waypoint {waypoint_order[i] + 1}')
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            feedback = navigator.getFeedback()

            if feedback:
                print('Executing current waypoint: ' +
                      str(waypoint_order[i] + 1))
                now = navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600):
                    navigator.cancelTask()

        #Check for the waypoint result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Goal {waypoint_order[i] + 1} succeeded!')

            if sys.argv[1] == 'aruco' and waypoint_order[i] == 0:
                node.aruco_detected = False
                while not node.aruco_detected:
                    rclpy.spin_once(node)
                
                node.get_logger().warn("ARUCO")

        elif result == TaskResult.CANCELED:
            print(f'Goal {waypoint_order[i] + 1} was canceled!')
        elif result == TaskResult.FAILED:
            print(f'Goal {waypoint_order[i] + 1} failed!')
        else:
            print(f'Goal {waypoint_order[i] + 1} has an invalid return status!')

    exit(0)


if __name__ == '__main__':
    main()
