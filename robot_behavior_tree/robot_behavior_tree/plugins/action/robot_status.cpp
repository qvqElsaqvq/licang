//
// Created by Elsa on 24-10-6.
//
#include <string>

#include "robot_behavior_tree/plugins/action/robot_status.hpp"

namespace nav2_behavior_tree
{

    RobotStatusAction::RobotStatusAction(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(condition_name, conf),
        is_adjust(false),
        is_finished(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<bool>("is_adjust", is_adjust);
        config().blackboard->get<bool>("is_finished", is_finished);
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        robotstatus_sub_ = node_->create_subscription<robot_serial::msg::Robotstatus>(
            "/robot/robotstatus",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RobotStatusAction::robotstatusCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus RobotStatusAction::tick()
    {
        callback_group_executor_.spin_some();
        config().blackboard->get<bool>("is_adjust", is_adjust);
        config().blackboard->get<bool>("is_finished", is_finished);

        return BT::NodeStatus::SUCCESS;
    }

    void RobotStatusAction::robotstatusCallback(robot_serial::msg::Robotstatus::SharedPtr msg)
    {
        if(msg->is_adjust == 0x00){
            is_adjust = false;
            config().blackboard->set<bool>("is_adjust", is_adjust);
        }
        else if(msg->is_adjust == 0x01){
            is_adjust = true;
            config().blackboard->set<bool>("is_adjust", is_adjust);
        }
        if(msg->is_finished == 0x00){
            is_finished = false;
            config().blackboard->set<bool>("is_finished", is_finished);
        }
        else if(msg->is_finished == 0x01){
            is_finished = true;
            config().blackboard->set<bool>("is_finished", is_finished);
        }
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RobotStatusAction>("RobotStatus");
}