#include <string>

#include "robot_behavior_tree/plugins/action/update_goal.hpp"

namespace nav2_behavior_tree
{

    UpdateGoalAction::UpdateGoalAction(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        position_x(0.0),
        position_y(0.0),
        is_match_finish(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<std::string>("cmd_status", cmd_status);
    }

    BT::NodeStatus UpdateGoalAction::tick()
    {
        config().blackboard->get<bool>("is_match_finish", is_match_finish);
        if(!is_match_finish)
        {
            config().blackboard->get<std::string>("cmd_status", cmd_status);
            if(cmd_status == "avoid")
            {
                config().blackboard->get<double>("target_x1", position_x);
                config().blackboard->get<double>("target_y1", position_y);
            }
            else if(cmd_status == "rotating")
            {
                config().blackboard->get<double>("target_x2", position_x);
                config().blackboard->get<double>("target_y2", position_y);
            }
            else if(cmd_status == "platform")
            {
                config().blackboard->get<double>("target_x3", position_x);
                config().blackboard->get<double>("target_y3", position_y);
            }
            else if(cmd_status == "column")
            {
                config().blackboard->get<double>("target_x4", position_x);
                config().blackboard->get<double>("target_y4", position_y);
            }
            else if(cmd_status == "warehouse")
            {
                config().blackboard->get<double>("target_x5", position_x);
                config().blackboard->get<double>("target_y5", position_y);
            }
        }
        else
        {
            config().blackboard->get<double>("init_x", position_x);
            config().blackboard->get<double>("init_y", position_y);
        }

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
        std::cout << "当前状态 " << cmd_status << std::endl;
        std::cout<<"更新目标点"<<position_x<<" , "<<position_y<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::UpdateGoalAction>("UpdateGoal");
}
