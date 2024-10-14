//
// Created by elsa on 24-10-14.
//

#ifndef RECEIVE_INIT_ERROR_HPP
#define RECEIVE_INIT_ERROR_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robot_serial/msg/location.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class ReceiveInitErrorAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        ReceiveInitErrorAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        ReceiveInitErrorAction() = delete;

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
            };
        }

    private:
        double init_x_;
        double init_y_;
        double init_yaw_;
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
        double target_x1;
        double target_x2;
        double target_x3;
        double target_x4;
        double target_x5;
        double target_y1;
        double target_y2;
        double target_y3;
        double target_y4;
        double target_y5;
        int count;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_position_sub_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        geometry_msgs::msg::PoseStamped init_position;

        void init_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void calculate_new_position();
    };

} // namespace nav2_behavior_tree

#endif //RECEIVE_INIT_ERROR_HPP
