//
// Created by Elsa on 24-10-6.
//
#include <string>

#include "robot_behavior_tree/plugins/action/match_status.hpp"

namespace nav2_behavior_tree
{

    MatchStatusAction::MatchStatusAction(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(condition_name, conf),
        is_match_finish(false)
    {
        config().blackboard->set<bool>("is_match_finish", false);
    }

    BT::NodeStatus MatchStatusAction::tick()
    {
        is_match_finish = true;
        config().blackboard->set<bool>("is_match_finish", is_match_finish);

        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::MatchStatusAction>("MatchStatus");
}