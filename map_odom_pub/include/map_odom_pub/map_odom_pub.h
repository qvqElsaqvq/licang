//
// Created by elsa on 24-7-30.
//

#ifndef MAP_ODOM_PUB_H
#define MAP_ODOM_PUB_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

class MapOdomPublishNode : public rclcpp::Node
{
public:
    explicit MapOdomPublishNode();

private:
    std::string global_frame_id_;
    std::string odom_frame_id_;
    std::vector<double> initial_trans_;
    std::vector<double> initial_rot_;

    geometry_msgs::msg::TransformStamped init_transform_;
    std::mutex init_transform_mtx_;
    tf2_ros::TransformBroadcaster broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();
};

#endif //MAP_ODOM_PUB_H
