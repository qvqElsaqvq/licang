//
// Created by Elsa on 24-10-6.
//
#include "robot_behavior_tree/plugins/condition/if_match_finish.hpp"

namespace nav2_behavior_tree
{

    IfMatchFinishCondition::IfMatchFinishCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          is_match_finish(false)
    {
        config().blackboard->get<bool>("is_match_finish", is_match_finish);
    }

    BT::NodeStatus IfMatchFinishCondition::tick()
    {
        config().blackboard->get<bool>("is_match_finish", is_match_finish);
        if (!is_match_finish)
        {
            std::cout<<"比赛未结束！"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        std::cout<<"比赛结束，回到出发点"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfMatchFinishCondition>("IfMatchFinish");
}