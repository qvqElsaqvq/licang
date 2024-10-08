#ifndef PRINTF_HPP_
#define PRINTF_HPP_
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class PrintfAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        PrintfAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        PrintfAction() = delete;

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
                BT::InputPort<std::string>("txt", "txt"),
            };
         }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        std::string txt;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
