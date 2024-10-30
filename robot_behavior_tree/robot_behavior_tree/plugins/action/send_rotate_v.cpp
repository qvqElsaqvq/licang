//
// Created by elsa on 24-10-23.
//
#include "robot_behavior_tree/plugins/action/send_rotate_v.hpp"
#include <chrono>

namespace nav2_behavior_tree
{

    SendRotateVAction::SendRotateVAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        if_send_rotate_v(false),
        image_x_last(140),
        distance(60.0),
        cnt(0)
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_);
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        image_location_sub_ = node_->create_subscription<robot_serial::msg::Imagelocation>("/image_location", 10,
                                     std::bind(&SendRotateVAction::imagelocationCallback, this, std::placeholders::_1),
                                     sub_option);
        velocity_pub_ = node_->create_publisher<robot_serial::msg::RotateVelocity>("/robot/velocity", 10);
        config().blackboard->get<bool>("if_send_rotate_v",if_send_rotate_v);
    }

    BT::NodeStatus SendRotateVAction::tick()
    {
        config().blackboard->get<bool>("if_send_rotate_v",if_send_rotate_v);
        if(if_send_rotate_v){
            return BT::NodeStatus::SUCCESS;
        }
        auto start_time = clock_.now();
        auto end_time = clock_.now();
        double seconds;
        while(!if_send_rotate_v){
            callback_group_executor_.spin_some();
            if(image_x_next - image_x_last > 15){
                if(cnt == 0){
                    start_time = clock_.now();
                    cnt++;
                }
                else{
                    end_time = clock_.now();
                    auto duration = end_time - start_time;
                    seconds = duration.seconds();  // 将时间间隔转换为秒
                    robot_serial::msg::RotateVelocity rotate_v;
                    rotate_v.rotate_velocity = distance / seconds;
                    for(int i = 0; i < 10; i++)
                        velocity_pub_->publish(rotate_v);
                    std::cout << "圆盘机转速为：" << rotate_v.rotate_velocity << std::endl;

                    if_send_rotate_v = true;
                    config().blackboard->set<bool>("if_send_rotate_v",if_send_rotate_v);
                    break;
                }
            }
            image_x_last = image_x_next;
        }

        return BT::NodeStatus::SUCCESS;
    }

    void SendRotateVAction::imagelocationCallback(const robot_serial::msg::Imagelocation::SharedPtr msg){
        image_x_next = msg->image_x;
        image_y_next = msg->image_y;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendRotateVAction>("SendRotateV");
}
