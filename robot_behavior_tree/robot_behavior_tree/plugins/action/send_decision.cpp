#include <string>

#include "robot_behavior_tree/plugins/action/send_decision.hpp"

namespace nav2_behavior_tree
{

    SendDecisionAction::SendDecisionAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        if_navigation(0),
        catch_decision(0),
        qrcode_number(0)
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_ );
        decision_pub_ = node_->create_publisher<robot_serial::msg::Decision>("/robot/decision", 10);
        getInput("if_navigation",if_navigation);
        getInput("catch_decision",catch_decision);
        getInput("qrcode_number",qrcode_number);
    }

    BT::NodeStatus SendDecisionAction::tick()
    {
        getInput("if_navigation",if_navigation);
        getInput("catch_decision",catch_decision);
        getInput("qrcode_number",qrcode_number);
        robot_serial::msg::Decision decision;

        decision.catch_decision = catch_decision;;
        decision.qrcode_number = qrcode_number;
        decision.if_navigation = if_navigation;
        decision_pub_->publish(decision);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendDecisionAction>("SendDecision");
}
