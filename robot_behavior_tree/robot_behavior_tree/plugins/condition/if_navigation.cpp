//
// Created by Elsa on 24-10-6.
//
#include "robot_behavior_tree/plugins/condition/if_navigation.hpp"

namespace nav2_behavior_tree
{

    IfNavigationCondition::IfNavigationCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          if_navigation(1)
    {
        config().blackboard->get<int>("if_navigation", if_navigation);
    }

    BT::NodeStatus IfNavigationCondition::tick()
    {
        config().blackboard->get<int>("if_navigation", if_navigation);
        if (if_navigation)
        {
            std::cout<<"处于导航模式"<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        std::cout<<"未处于导航模式"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfNavigationCondition>("IfNavigation");
}