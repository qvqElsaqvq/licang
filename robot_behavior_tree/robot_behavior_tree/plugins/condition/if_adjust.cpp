//
// Created by Elsa on 24-10-6.
//
#include "robot_behavior_tree/plugins/condition/if_adjust.hpp"

namespace nav2_behavior_tree
{

    IfAdjustCondition::IfAdjustCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          is_adjust(false)
    {
        config().blackboard->get<bool>("is_adjust", is_adjust);
        config().blackboard->set<bool>("is_openmv_open", false);
    }

    BT::NodeStatus IfAdjustCondition::tick()
    {
        config().blackboard->get<bool>("is_adjust", is_adjust);
        if (is_adjust)
        {
            std::cout<<"微调完成，准备夹球"<<std::endl;
            config().blackboard->set<bool>("is_openmv_open", true);
            return BT::NodeStatus::SUCCESS;
        }
        config().blackboard->set<bool>("is_openmv_open", false);
        std::cout<<"微调未完成，等待中"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfAdjustCondition>("IfAdjust");
}