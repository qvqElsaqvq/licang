//
// Created by elsa on 24-10-17.
//

#include "robot_behavior_tree/plugins/action/set_goal.hpp"

namespace nav2_behavior_tree
{

    SetGoalAction::SetGoalAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        init_x_(0.0),
        init_y_(0.0)
    {
        getInput("position_x1",position_x1);
        getInput("position_y1",position_y1);
        getInput("position_x2",position_x2);
        getInput("position_y2",position_y2);
        getInput("position_x3",position_x3);
        getInput("position_y3",position_y3);
        getInput("position_x4",position_x4);
        getInput("position_y4",position_y4);
        getInput("position_x5",position_x5);
        getInput("position_y5",position_y5);
        config().blackboard->set<double>("init_x", init_x_);
        config().blackboard->set<double>("init_y", init_y_);
        config().blackboard->set<double>("target_x1", 0.0);
        config().blackboard->set<double>("target_y1", 0.0);
        config().blackboard->set<double>("target_x2", 0.0);
        config().blackboard->set<double>("target_y2", 0.0);
        config().blackboard->set<double>("target_x3", 0.0);
        config().blackboard->set<double>("target_y3", 0.0);
        config().blackboard->set<double>("target_x4", 0.0);
        config().blackboard->set<double>("target_y4", 0.0);
        config().blackboard->set<double>("target_x5", 0.0);
        config().blackboard->set<double>("target_y5", 0.0);
    }

    BT::NodeStatus SetGoalAction::tick()
    {
        //获取消除误差前的初始目标点
        getInput("position_x1",position_x1);
        getInput("position_y1",position_y1);
        getInput("position_x2",position_x2);
        getInput("position_y2",position_y2);
        getInput("position_x3",position_x3);
        getInput("position_y3",position_y3);
        getInput("position_x4",position_x4);
        getInput("position_y4",position_y4);
        getInput("position_x5",position_x5);
        getInput("position_y5",position_y5);

        init_x_ = -0.16;
        init_y_ = -0.16;
        config().blackboard->set<double>("init_x", init_x_);
        config().blackboard->set<double>("init_y", init_y_);
        config().blackboard->set<double>("target_x1", position_x1);
        config().blackboard->set<double>("target_y1", position_y1);
        config().blackboard->set<double>("target_x2", position_x2);
        config().blackboard->set<double>("target_y2", position_y2);
        config().blackboard->set<double>("target_x3", position_x3);
        config().blackboard->set<double>("target_y3", position_y3);
        config().blackboard->set<double>("target_x4", position_x4);
        config().blackboard->set<double>("target_y4", position_y4);
        config().blackboard->set<double>("target_x5", position_x5);
        config().blackboard->set<double>("target_y5", position_y5);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetGoalAction>("SetGoal");
}
