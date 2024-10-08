//
// Created by Elsa on 24-10-8.
//

#ifndef CHANGE_GOAL_HPP
#define CHANGE_GOAL_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class ChangeGoalAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        ChangeGoalAction(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        ChangeGoalAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("current_decision", "change current goal to another")
            };
        }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        int current_decision;
        std::string cmd_status;  //0-avoid,1-rotating,2-platform,3-column,4-warehouse
    };

} // namespace nav2_behavior_tree

#endif //CHANGE_GOAL_HPP
