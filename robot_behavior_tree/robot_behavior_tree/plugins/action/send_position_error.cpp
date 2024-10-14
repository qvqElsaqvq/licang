#include <string>
#include "robot_behavior_tree/plugins/action/send_position_error.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace nav2_behavior_tree
{

    SendPositionErrorAction::SendPositionErrorAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        x_error_(0.0),
        y_error_(0.0),
        angle_error_(0.0),
        global_frame_("map"),
        robot_base_frame_("base_link")
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_ );
        tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        position_err_pub_ = node_->create_publisher<robot_serial::msg::Location>("/robot/location_error", 10);
    }

    BT::NodeStatus SendPositionErrorAction::tick()
    {
        getInput("goal",current_goal);          //获取目标点坐标
        getInput("target_angle",target_angle_); //获取目标点角度
        //获取当前坐标和角度
        if (!nav2_util::getCurrentPose(current_position, *tf_, global_frame_, robot_base_frame_))
        {
            std::cout<<"没有获取到TF变换"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        tf2::Quaternion quaternion(current_position.pose.orientation.x, current_position.pose.orientation.y,
                                   current_position.pose.orientation.z, current_position.pose.orientation.w);
        tf2::Matrix3x3 euler(quaternion);
        double roll, pitch, yaw;
        euler.getRPY(roll, pitch, yaw);
        //std::cout << "yaw: " << yaw << std::endl;
        //计算坐标偏差(目标 - 当前)并发送
        robot_serial::msg::Location location_error;
        x_error_ = (float)(current_goal.pose.position.x - current_position.pose.position.x);
        y_error_ = (float)(current_goal.pose.position.y - current_position.pose.position.y);
        angle_error_ = (float)(target_angle_ - yaw);
        std::cout << "x_error: " << x_error_ << " y_error: " << y_error_ <<  " angle_error: " << angle_error_ << std::endl;
        location_error.x_err = x_error_;
        location_error.y_err = y_error_;
        location_error.angle_err = angle_error_;
        position_err_pub_->publish(location_error);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendPositionErrorAction>("SendPositionError");
}
