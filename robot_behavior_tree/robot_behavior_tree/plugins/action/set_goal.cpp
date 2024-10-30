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
        init_x_(-0.16),
        init_y_(-0.16),
        cmd_status("avoid"),
        if_navigation(1)
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
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<std::string>("cmd_status", cmd_status);
        config().blackboard->set<int>("if_navigation",if_navigation );
        decision_pub_ = node_->create_publisher<robot_serial::msg::Decision>("/robot/decision", 10);
    }

    BT::NodeStatus SetGoalAction::tick()
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

        init_x_ = -0.17;
        init_y_ = -0.22;
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

        config().blackboard->get<std::string>("cmd_status", cmd_status);
        if(cmd_status == "avoid")
        {
            pose.header.stamp = node_->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = position_x1;
            pose.pose.position.y = position_y1;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            setOutput<geometry_msgs::msg::PoseStamped>("goal",pose);
            std::cout<<"初始目标点"<<position_x1<<" , "<<position_y1<<std::endl;
            if_navigation = 1;
            config().blackboard->set<int>("if_navigation",if_navigation);
            robot_serial::msg::Decision decision;

            decision.catch_decision = 0;
            decision.qrcode_number = 0;
            decision.if_navigation = if_navigation;
            decision_pub_->publish(decision);

            config().blackboard->set<bool>("if_send_rotate_v",false);
        }

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetGoalAction>("SetGoal");
}
