//
// Created by elsa on 24-10-23.
//

#ifndef SEND_ROTATE_V_HPP
#define SEND_ROTATE_V_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robot_serial/msg/imagelocation.hpp"
#include "robot_serial/msg/rotate_velocity.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SendRotateVAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendRotateVAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendRotateVAction() = delete;

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
        int image_x_last;        //识别图像中心x坐标
        int image_x_next;        //识别图像中心x坐标
        int image_y_next;
        double distance;            //球走过的距离（提前算好）
        int cnt;
        bool if_send_rotate_v;   //是否已经发送圆盘机转速
        rclcpp::Subscription<robot_serial::msg::Imagelocation>::SharedPtr image_location_sub_;
        rclcpp::Publisher<robot_serial::msg::RotateVelocity>::SharedPtr velocity_pub_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Clock clock_;
        void imagelocationCallback(const robot_serial::msg::Imagelocation::SharedPtr msg);
    };

} // namespace nav2_behavior_tree

#endif //SEND_ROTATE_V_HPP
