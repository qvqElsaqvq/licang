#ifndef SEND_DECISION_HPP_
#define SEND_DECISION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robot_serial/msg/decision.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SendDecisionAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendDecisionAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendDecisionAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<uint8_t>("if_navigation", "if is navigating"),
                BT::InputPort<uint8_t>("catch_decision", "if need catch ball"),
                BT::InputPort<uint8_t>("qrcode_number", "the number of qrcode"),
            };
        }

    private:
        uint8_t if_navigation;   //是否在导航模式，0为不在，1为在
        uint8_t  catch_decision; //是否拨球，0为不拨，1为拨
        uint8_t qrcode_number;   //二维码识别到的数字
        rclcpp::Publisher<robot_serial::msg::Decision>::SharedPtr decision_pub_;
        rclcpp::Node::SharedPtr node_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
