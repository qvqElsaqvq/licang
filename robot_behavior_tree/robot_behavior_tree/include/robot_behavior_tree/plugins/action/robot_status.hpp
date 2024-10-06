//
// Created by Elsa on 24-10-6.
//

#ifndef ROBOT_STATUS_HPP
#define ROBOT_STATUS_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robot_serial/msg/robotstatus.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class RobotStatusAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RobotStatusAction(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        RobotStatusAction() = delete;

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
            return {};
         }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        void robotstatusCallback(robot_serial::msg::Robotstatus::SharedPtr msg);
        bool is_adjust;
        bool is_receive_0x14;
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<robot_serial::msg::Robotstatus>::SharedPtr robotstatus_sub_;
    };

} // namespace nav2_behavior_tree

#endif //ROBOT_STATUS_HPP
