//
// Created by Elsa on 24-10-6.
//
#include <string>

#include "robot_behavior_tree/plugins/action/change_goal.hpp"

namespace nav2_behavior_tree
{

    ChangeGoalAction::ChangeGoalAction(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(condition_name, conf),
        current_decision(0),
        cmd_status("avoid")
    {
        config().blackboard->set<std::string>("cmd_status", cmd_status);
        getInput("current_decision",current_decision);
    }

    BT::NodeStatus ChangeGoalAction::tick()
    {
        getInput("current_decision",current_decision);
        std::cout << "当前目标为： " << current_decision << std::endl;
        if(current_decision == 0)
        {
            cmd_status = "avoid";
        }
        else if(current_decision == 1)
        {
            cmd_status = "rotating";
        }
        else if(current_decision == 2)
        {
            cmd_status = "platform";
        }
        else if(current_decision == 3)
        {
            cmd_status = "column";
        }
        else if(current_decision == 4)
        {
            cmd_status = "warehouse";
        }
        config().blackboard->set<std::string>("cmd_status", cmd_status);

        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::ChangeGoalAction>("ChangeGoal");
}