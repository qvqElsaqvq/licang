//
// Created by Elsa on 24-10-6.
//
#include "robot_behavior_tree/plugins/condition/if_catch_ball.hpp"

namespace nav2_behavior_tree
{

    IfCatchBallCondition::IfCatchBallCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_);
        decision_pub_ = node_->create_publisher<robot_serial::msg::Decision>("/robot/decision", 10);
        image_location_sub_ = node_->create_subscription<robot_serial::msg::Imagelocation>("/image_location", 10,
                                    std::bind(&IfCatchBallCondition::imagelocationCallback, this, std::placeholders::_1));
        getInput("image_x_threshold_min", image_x_threshold_min);
        getInput("image_x_threshold_max", image_x_threshold_max);
        getInput("image_y_threshold_min", image_y_threshold_min);
        getInput("image_y_threshold_max", image_y_threshold_max);
    }

    BT::NodeStatus IfCatchBallCondition::tick()
    {
        getInput("image_x_threshold_min", image_x_threshold_min);
        getInput("image_x_threshold_max", image_x_threshold_max);
        getInput("image_y_threshold_min", image_y_threshold_min);
        getInput("image_y_threshold_max", image_y_threshold_max);
        robot_serial::msg::Decision decision;
        if (image_x <= image_x_threshold_max && image_x >= image_x_threshold_min &&
            image_y <= image_y_threshold_max && image_y >= image_y_threshold_min)
        {
            decision.if_navigation = 0x00;
            decision.catch_decision = 0x01;
            decision.qrcode_number = 0x00;
            decision_pub_->publish(decision);
            std::cout<<"--------拨球--------"<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        decision.if_navigation = 0x00;
        decision.catch_decision = 0x00;
        decision.qrcode_number = 0x00;
        decision_pub_->publish(decision);
        std::cout<<"--------等待--------"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }

    void IfCatchBallCondition::imagelocationCallback(const robot_serial::msg::Imagelocation::SharedPtr msg){
        image_x = msg->image_x;
        image_y = msg->image_y;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfCatchBallCondition>("IfCatchBall");
}