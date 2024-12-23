#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include "math.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/buffer.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArucoTransformPublisher : public rclcpp::Node
{
public:
    ArucoTransformPublisher()
        : Node("aruco_transform_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          static_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this))
    {
        // Subscriber to ArUco pose (wrt camera_link)
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10,
            std::bind(&ArucoTransformPublisher::pose_callback, this, std::placeholders::_1));

        // Publisher for transformed pose
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pose_in_map", 10);

        RCLCPP_INFO(this->get_logger(), "ArUco Transform Publisher started.");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        try
        {
            // Transform from camera_link to map frame
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform("map", pose_msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

            geometry_msgs::msg::PoseStamped pose_in_map;
            tf2::doTransform(*pose_msg, pose_in_map, transform_stamped);

            RCLCPP_INFO(this->get_logger(), "ArUco Pose in map frame: x: %.2f, y: %.2f, z: %.2f",
                        pose_in_map.pose.position.x,
                        pose_in_map.pose.position.y,
                        pose_in_map.pose.position.z);

            // Publisher of transformed pose
            pose_in_map.header.frame_id = "map";
            pose_in_map.header.stamp = this->now();
            publisher_->publish(pose_in_map);

            // Publish the static transform only the first time the aruco is detected
            if (!aruco_static_tf_published_)
            {
                publish_static_transform(pose_in_map);
                aruco_static_tf_published_ = true;
                RCLCPP_INFO(this->get_logger(), "Static TF for ArUco marker published.");
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform pose: %s", ex.what());
        }
    }

    void publish_static_transform(const geometry_msgs::msg::PoseStamped &pose)
    {
        geometry_msgs::msg::TransformStamped static_transform_stamped;

        static_transform_stamped.header.stamp = this->now();
        static_transform_stamped.header.frame_id = "map";  
        static_transform_stamped.child_frame_id = "aruco_marker_static";  

       
        static_transform_stamped.transform.translation.x = pose.pose.position.x;
        static_transform_stamped.transform.translation.y = pose.pose.position.y;
        static_transform_stamped.transform.translation.z = pose.pose.position.z;

        
        static_transform_stamped.transform.rotation = pose.pose.orientation;

        static_broadcaster_->sendTransform(static_transform_stamped);
    }


    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    bool aruco_static_tf_published_ = false;  
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoTransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

