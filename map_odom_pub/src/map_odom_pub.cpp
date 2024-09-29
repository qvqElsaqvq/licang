//
// Created by elsa on 24-7-31.
//
#include "map_odom_pub/map_odom_pub.h"

using namespace std::chrono_literals;

MapOdomPublishNode::MapOdomPublishNode()
    : Node("map_odom_pub_node"),
      broadcaster_(this)
{
    RCLCPP_INFO(get_logger(), "node is created.");
    // 声明参数
    declare_parameter("global_frame_id", "map");
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("initial_trans", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("initial_rot", std::vector<double>{0.0, 0.0, 0.0, 1.0});

    get_parameter("global_frame_id", global_frame_id_);
    get_parameter("odom_frame_id", odom_frame_id_);
    get_parameter("initial_trans", initial_trans_);
    get_parameter("initial_rot", initial_rot_);

    // 初始化订阅者、发布者、服务端、客户端
    timer_ = this->create_wall_timer( 0.05s, std::bind(&MapOdomPublishNode::timer_callback, this));
    RCLCPP_INFO(get_logger(), "初始位姿初始化完成");
}

void MapOdomPublishNode::timer_callback()
{
    // 发布变换
    init_transform_mtx_.lock();
    init_transform_.header.stamp = now();
    init_transform_.header.frame_id = global_frame_id_;
    init_transform_.child_frame_id = odom_frame_id_;
    init_transform_.transform.translation.x = initial_trans_[0];
    init_transform_.transform.translation.y = initial_trans_[1];
    init_transform_.transform.translation.z = initial_trans_[2];
    init_transform_.transform.rotation.x = initial_rot_[0];
    init_transform_.transform.rotation.y = initial_rot_[1];
    init_transform_.transform.rotation.z = initial_rot_[2];
    init_transform_.transform.rotation.w = initial_rot_[3];
    broadcaster_.sendTransform(init_transform_);
    //RCLCPP_INFO(get_logger(), "translation: [ %f, %f, %f], rotation: [ %f, %f, %f, %f]", initial_trans_[0], initial_trans_[1],
    //    initial_trans_[2], initial_rot_[0], initial_rot_[1], initial_rot_[2], initial_rot_[3]);
    init_transform_mtx_.unlock();
}