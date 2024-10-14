//
// Created by Elsa on 24-10-13.
//

#ifndef SEND_POSITION_ERROR_HPP
#define SEND_POSITION_ERROR_HPP

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
    class SendPositionErrorAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendPositionErrorAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendPositionErrorAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "goal"),
                BT::InputPort<float>("target_angle", "target angle")
            };
        }

    private:
        float target_angle_;
        float x_error_;
        float y_error_;
        float angle_error_;
        rclcpp::Publisher<robot_serial::msg::Location>::SharedPtr position_err_pub_;
        rclcpp::Node::SharedPtr node_;
        std::string global_frame_;
        std::string robot_base_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        geometry_msgs::msg::PoseStamped current_goal;
        geometry_msgs::msg::PoseStamped current_position;
    };

} // namespace nav2_behavior_tree

#endif //SEND_POSITION_ERROR_HPP
