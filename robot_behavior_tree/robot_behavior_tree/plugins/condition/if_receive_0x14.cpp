//
// Created by Elsa on 24-10-6.
//
#include "robot_behavior_tree/plugins/condition/if_receive_0x14.hpp"

namespace nav2_behavior_tree
{

    IfReceive0x14Condition::IfReceive0x14Condition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          is_receive_0x14(false)
    {
        config().blackboard->get<bool>("is_receive_0x14", is_receive_0x14);
    }

    BT::NodeStatus IfReceive0x14Condition::tick()
    {
        config().blackboard->get<bool>("is_receive_0x14", is_receive_0x14);
        if (is_receive_0x14)
        {
            std::cout<<"干扰球已夹取"<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        std::cout<<"干扰球未夹取"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfReceive0x14Condition>("IfReceive0x14");
}