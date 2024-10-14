//
// Created by elsa on 24-7-31.
//
#include "map_odom_pub/map_odom_pub.h"

using namespace std::chrono_literals;

MapOdomPublishNode::MapOdomPublishNode()
    : Node("map_odom_pub_node")
{
    RCLCPP_INFO(get_logger(), "map_odom_pub node is created.");
    flag = true;

    init_transform_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot/init_transform", 1);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer( 30s, std::bind(&MapOdomPublishNode::timer_callback, this));
    RCLCPP_INFO(get_logger(), "map_odom_pub 初始化完成");
}

void MapOdomPublishNode::timer_callback()
{
    if(flag)
    {
        double roll, pitch, yaw;
        try
        {
            init_transform_ = tf_buffer_->lookupTransform("map", "livox", tf2::TimePointZero);
            tf2::Quaternion quaternion(init_transform_.transform.translation.x,
                init_transform_.transform.translation.y,
                init_transform_.transform.translation.z,
                init_transform_.transform.rotation.w);
            tf2::Matrix3x3 euler(quaternion);
            euler.getRPY(roll, pitch, yaw);
            std::cout << "----------------------------------------------------" << std::endl;
            std::cout << "----------------------------------------------------" << std::endl;
            std::cout << "----------------------------------------------------" << std::endl;
            std::cout << "----------------------------------------------------" << std::endl;
            std::cout << "----------------------------------------------------" << std::endl;
            std::cout << "init_transform: x: " << init_transform_.transform.translation.x
            << " y: " << init_transform_.transform.translation.y << " yaw:" << yaw << std::endl;

            geometry_msgs::msg::PoseStamped robot_transform;
            robot_transform.header.stamp = init_transform_.header.stamp;
            robot_transform.header.frame_id = init_transform_.header.frame_id;
            robot_transform.pose.position.x = init_transform_.transform.translation.x;
            robot_transform.pose.position.y = init_transform_.transform.translation.y;
            robot_transform.pose.position.z = init_transform_.transform.translation.z;
            robot_transform.pose.orientation = init_transform_.transform.rotation;
            init_transform_pub_->publish(robot_transform);

            flag = false;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(get_logger(), "%s", ex.what());
            return;
        }
    }
}
