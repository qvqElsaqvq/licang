//
// Created by Elsa on 24-10-6.
//

#ifndef IF_CATCH_BALL_HPP
#define IF_CATCH_BALL_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robot_serial/msg/decision.hpp"
#include "robot_serial/msg/imagelocation.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IfCatchBallCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IfCatchBallCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfCatchBallCondition() = delete;

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
                BT::InputPort<int>("image_x_threshold_min", "image x location minimun threshold"),
                BT::InputPort<int>("image_x_threshold_max", "image x location maximun threshold"),
                BT::InputPort<int>("image_y_threshold_min", "image y location minimun threshold"),
                BT::InputPort<int>("image_y_threshold_max", "image y location maximun threshold"),
            };
        }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        int image_x;             //识别图像中心x坐标
        int image_y;             //识别图像中心y坐标
        int image_x_threshold_min;   //识别图像中心x坐标最小阈值
        int image_x_threshold_max;   //识别图像中心x坐标最大阈值
        int image_y_threshold_min;   //识别图像中心y坐标最小阈值
        int image_y_threshold_max;   //识别图像中心y坐标最大阈值
        rclcpp::Publisher<robot_serial::msg::Decision>::SharedPtr decision_pub_;
        rclcpp::Subscription<robot_serial::msg::Imagelocation>::SharedPtr image_location_sub_;
        rclcpp::Node::SharedPtr node_;
        void imagelocationCallback(const robot_serial::msg::Imagelocation::SharedPtr msg);
    };

} // namespace nav2_behavior_tree

#endif //IF_CATCH_BALL_HPP
