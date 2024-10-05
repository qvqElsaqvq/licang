#include <string>

#include "robot_behavior_tree/plugins/action/update_goal.hpp"

namespace nav2_behavior_tree
{

    UpdateGoalAction::UpdateGoalAction(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
       position_x(0.0),
       position_y(0.0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        getInput("position_x",position_x);
        getInput("position_y",position_y);
    }

    BT::NodeStatus UpdateGoalAction::tick()
    {
        getInput("position_x",position_x);
        getInput("position_y",position_y);
        pose.header.stamp = node_->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = position_x;
        pose.pose.position.y = position_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        setOutput<geometry_msgs::msg::PoseStamped>("goal",pose);
        std::cout<<"更新目标点"<<position_x<<" , "<<position_y<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::UpdateGoalAction>("UpdateGoal");
}
