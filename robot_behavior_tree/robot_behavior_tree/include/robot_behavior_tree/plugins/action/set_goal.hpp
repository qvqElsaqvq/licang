//
// Created by elsa on 24-10-17.
//

#ifndef SET_GOAL_HPP
#define SET_GOAL_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robot_serial/msg/decision.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SetGoalAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SetGoalAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SetGoalAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
            BT::InputPort<double>("position_x1", "position_x to plan to"),
            BT::InputPort<double>("position_y1", "position_y to plan to"),
            BT::InputPort<double>("position_x2", "position_x to plan to"),
            BT::InputPort<double>("position_y2", "position_y to plan to"),
            BT::InputPort<double>("position_x3", "position_x to plan to"),
            BT::InputPort<double>("position_y3", "position_y to plan to"),
            BT::InputPort<double>("position_x4", "position_x to plan to"),
            BT::InputPort<double>("position_y4", "position_y to plan to"),
            BT::InputPort<double>("position_x5", "position_x to plan to"),
            BT::InputPort<double>("position_y5", "position_y to plan to"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
            };
        }

    private:
        double init_x_;
        double init_y_;
        double position_x1;
        double position_y1;
        double position_x2;
        double position_y2;
        double position_x3;
        double position_y3;
        double position_x4;
        double position_y4;
        double position_x5;
        double position_y5;
        int if_navigation;
        std::string cmd_status;
        rclcpp::Node::SharedPtr node_;
        geometry_msgs::msg::PoseStamped pose;
        rclcpp::Publisher<robot_serial::msg::Decision>::SharedPtr decision_pub_;
    };

} // namespace nav2_behavior_tree

#endif //SET_GOAL_HPP
