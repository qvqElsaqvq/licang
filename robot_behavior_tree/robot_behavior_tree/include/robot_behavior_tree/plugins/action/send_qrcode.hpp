//
// Created by Elsa on 24-10-7.
//

#ifndef SEND_QRCODE_HPP
#define SEND_QRCODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robot_serial/msg/decision.hpp"
#include "robot_serial/msg/qrcodeinfo.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SendQrcodeAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendQrcodeAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendQrcodeAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
        uint8_t qrcode_number;   //二维码识别到的数字
        rclcpp::Publisher<robot_serial::msg::Decision>::SharedPtr decision_pub_;
        rclcpp::Subscription<robot_serial::msg::Qrcodeinfo>::SharedPtr qrcode_sub_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        void qrcodeCallback(const robot_serial::msg::Qrcodeinfo::SharedPtr msg);
    };

} // namespace nav2_behavior_tree

#endif //SEND_QRCODE_HPP
