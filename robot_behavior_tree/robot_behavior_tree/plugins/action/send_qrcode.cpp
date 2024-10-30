#include "robot_behavior_tree/plugins/action/send_qrcode.hpp"

namespace nav2_behavior_tree
{

    SendQrcodeAction::SendQrcodeAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        qrcode_number(0)
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_ );
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        decision_pub_ = node_->create_publisher<robot_serial::msg::Decision>("/robot/decision", 10);
        qrcode_sub_ = node_->create_subscription<robot_serial::msg::Qrcodeinfo>("/robot/qrcodeinfo", 1,
                                std::bind(&SendQrcodeAction::qrcodeCallback, this, std::placeholders::_1), sub_option);
    }

    BT::NodeStatus SendQrcodeAction::tick()
    {
        robot_serial::msg::Decision decision;
        callback_group_executor_.spin_some();

        decision.catch_decision = 0x00;
        decision.qrcode_number = qrcode_number;
        decision.if_navigation = 0x00;
        decision_pub_->publish(decision);

        return BT::NodeStatus::SUCCESS;
    }

    void SendQrcodeAction::qrcodeCallback(const robot_serial::msg::Qrcodeinfo::SharedPtr msg){
        if(msg->qrcode_ball == 0x31){
            qrcode_number = 0x11;
        }
        else if(msg->qrcode_ball == 0x32){
            qrcode_number = 0x12;
        }
        else if(msg->qrcode_ball == 0x33){
            qrcode_number = 0x13;
        }
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendQrcodeAction>("SendQrcode");
}
