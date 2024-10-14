//
// Created by elsa on 24-7-30.
//

#ifndef MAP_ODOM_PUB_H
#define MAP_ODOM_PUB_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"

class MapOdomPublishNode : public rclcpp::Node
{
public:
    explicit MapOdomPublishNode();

private:
    bool flag;

    geometry_msgs::msg::TransformStamped init_transform_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr init_transform_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();
};

#endif //MAP_ODOM_PUB_H
