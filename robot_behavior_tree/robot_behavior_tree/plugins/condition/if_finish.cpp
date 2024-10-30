//
// Created by Elsa on 24-10-6.
//
#include "robot_behavior_tree/plugins/condition/if_finish.hpp"

namespace nav2_behavior_tree
{

    IfFinishCondition::IfFinishCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          is_finished(false)
    {
        config().blackboard->set<bool>("is_openmv_open", false);
        config().blackboard->get<bool>("is_finished", is_finished);
    }

    BT::NodeStatus IfFinishCondition::tick()
    {
        config().blackboard->get<bool>("is_finished", is_finished);
        if (is_finished)
        {
            std::cout<<"取球已经完成"<<std::endl;
            config().blackboard->set<bool>("is_openmv_open", false);
            return BT::NodeStatus::SUCCESS;
        }
        std::cout<<"取球未完成，等待中"<<std::endl;
        config().blackboard->set<bool>("is_openmv_open", true);
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfFinishCondition>("IfFinish");
}