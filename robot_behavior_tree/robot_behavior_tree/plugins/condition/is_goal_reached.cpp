#include <string>

#include "robot_behavior_tree/plugins/condition/is_goal_reached.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace nav2_behavior_tree
{

    IsGoalReachedCondition::IsGoalReachedCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        global_frame_("map"),
        robot_base_frame_("livox"),
        is_goal_reached(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        config().blackboard->set<bool>("is_goal_reached",is_goal_reached);
        if (!node_->has_parameter("transform_tolerance"))
        {
            node_->declare_parameter("transform_tolerance", 0.15);
        }
        node_->get_parameter("transform_tolerance", transform_tolerance_);
    }

    BT::NodeStatus IsGoalReachedCondition::tick()
    {
        getInput("goal",current_goal);
        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(
                current_pose, *tf_, global_frame_, robot_base_frame_,
                transform_tolerance_))
        {
            std::cout<<"没有获取到TF变换"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        // tf2::Quaternion quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        // tf2::Matrix3x3 euler(quaternion);
        // double roll, pitch, yaw;
        // euler.getRPY(roll, pitch, yaw);
        if((fabs(current_goal.pose.position.x - current_pose.pose.position.x) <= 0.1)&&(fabs(current_goal.pose.position.y - current_pose.pose.position.y) <= 0.1))
        {
            is_goal_reached = true;
            config().blackboard->set<bool>("is_goal_reached",is_goal_reached);
            std::cout<<"当前机器人处于目标点 x: "<<current_goal.pose.position.x<<", y: "<<current_goal.pose.position.y<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else{
            is_goal_reached = false;
            config().blackboard->set<bool>("is_goal_reached",is_goal_reached);
            std::cout<<"当前机器人未处于目标点 x: "<<current_goal.pose.position.x<<", y: "<<current_goal.pose.position.y<<std::endl;
            return BT::NodeStatus::FAILURE;
        }  
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsGoalReachedCondition>("IsGoalReached");
}