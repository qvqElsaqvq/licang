#ifndef UPDATE_GOAL_HPP_
#define UPDATE_GOAL_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_serial/msg/decision.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class UpdateGoalAction : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        UpdateGoalAction(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        UpdateGoalAction() = delete;

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
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
            };
        }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        rclcpp::Node::SharedPtr node_;
        double position_x;
        double position_y;
        geometry_msgs::msg::PoseStamped pose;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        std::string cmd_status;  //0-avoid,1-rotating,2-platform,3-column,4-warehouse
        bool is_match_finish;
        int if_navigation;
        rclcpp::Publisher<robot_serial::msg::Decision>::SharedPtr decision_pub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
