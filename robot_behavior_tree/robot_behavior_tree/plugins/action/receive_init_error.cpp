#include <string>
#include "robot_behavior_tree/plugins/action/receive_init_error.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace nav2_behavior_tree
{

    ReceiveInitErrorAction::ReceiveInitErrorAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        init_x_(0.0),
        init_y_(0.0),
        init_yaw_(0.0),
        count(0),
        target_x1(0.0),
        target_x2(0.0),
        target_x3(0.0),
        target_x4(0.0),
        target_x5(0.0),
        target_y1(0.0),
        target_y2(0.0),
        target_y3(0.0),
        target_y4(0.0),
        target_y5(0.0)
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_ );
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;

        init_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/robot/init_transform",
            10, std::bind(&ReceiveInitErrorAction::init_position_callback, this, std::placeholders::_1),
            sub_option);

        getInput("position_x1",position_x1);
        getInput("position_y1",position_y1);
        getInput("position_x2",position_x2);
        getInput("position_y2",position_y2);
        getInput("position_x3",position_x3);
        getInput("position_y3",position_y3);
        getInput("position_x4",position_x4);
        getInput("position_y4",position_y4);
        getInput("position_x5",position_x5);
        getInput("position_y5",position_y5);
        config().blackboard->set<double>("init_x", init_x_);
        config().blackboard->set<double>("init_y", init_y_);
        config().blackboard->set<double>("target_x1", target_x1);
        config().blackboard->set<double>("target_y1", target_y1);
        config().blackboard->set<double>("target_x2", target_x2);
        config().blackboard->set<double>("target_y2", target_y2);
        config().blackboard->set<double>("target_x3", target_x3);
        config().blackboard->set<double>("target_y3", target_y3);
        config().blackboard->set<double>("target_x4", target_x4);
        config().blackboard->set<double>("target_y4", target_y4);
        config().blackboard->set<double>("target_x5", target_x5);
        config().blackboard->set<double>("target_y5", target_y5);
    }

    BT::NodeStatus ReceiveInitErrorAction::tick()
    {
        //获取消除误差前的初始目标点
        getInput("position_x1",position_x1);
        getInput("position_y1",position_y1);
        getInput("position_x2",position_x2);
        getInput("position_y2",position_y2);
        getInput("position_x3",position_x3);
        getInput("position_y3",position_y3);
        getInput("position_x4",position_x4);
        getInput("position_y4",position_y4);
        getInput("position_x5",position_x5);
        getInput("position_y5",position_y5);

        count++;
        //获取初始坐标和角度
        if (count < 10)
        {
            std::cout<<"等待初始TF变换发布中"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        callback_group_executor_.spin_some();

        tf2::Quaternion quaternion(init_position.pose.orientation.x, init_position.pose.orientation.y,
                                   init_position.pose.orientation.z, init_position.pose.orientation.w);
        tf2::Matrix3x3 euler(quaternion);
        double roll, pitch;
        euler.getRPY(roll, pitch, init_yaw_);
        init_x_ = init_position.pose.position.x;
        init_y_ = init_position.pose.position.y;
        config().blackboard->set<double>("init_x", init_x_);
        config().blackboard->set<double>("init_y", init_y_);

        //计算消除初始偏差后的目标点并进行设置
        calculate_new_position();

        return BT::NodeStatus::SUCCESS;
    }

    void ReceiveInitErrorAction::init_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        init_position.pose.position.x = msg->pose.position.x;
        init_position.pose.position.y = msg->pose.position.y;
        init_position.pose.orientation.x = msg->pose.orientation.x;
        init_position.pose.orientation.y = msg->pose.orientation.y;
        init_position.pose.orientation.z = msg->pose.orientation.z;
        init_position.pose.orientation.w = msg->pose.orientation.w;
    }

    void ReceiveInitErrorAction::calculate_new_position()
    {
        double radians = init_yaw_ * M_PI / 180.0;
        target_x1 = position_x1 * cos(radians) - position_y1 * sin(radians) + init_x_;
        target_y1 = position_x1 * sin(radians) + position_y1 * cos(radians) + init_y_;
        config().blackboard->set<double>("target_x1", target_x1);
        config().blackboard->set<double>("target_y1", target_y1);
        target_x2 = position_x2 * cos(radians) - position_y2 * sin(radians) + init_x_;
        target_y2 = position_x2 * sin(radians) + position_y2 * cos(radians) + init_y_;
        config().blackboard->set<double>("target_x2", target_x2);
        config().blackboard->set<double>("target_y2", target_y2);
        target_x3 = position_x3 * cos(radians) - position_y3 * sin(radians) + init_x_;
        target_y3 = position_x3 * sin(radians) + position_y3 + init_y_;
        config().blackboard->set<double>("target_x3", target_x3);
        config().blackboard->set<double>("target_y3", target_y3);
        target_x4 = position_x4 * cos(radians) - position_y4 * sin(radians) + init_x_;
        target_y4 = position_x4 * sin(radians) + position_y4 + init_y_;
        config().blackboard->set<double>("target_x4", target_x4);
        config().blackboard->set<double>("target_y4", target_y4);
        target_x5 = position_x5 * cos(radians) - position_y5 * sin(radians) + init_x_;
        target_y5 = position_x5 * sin(radians) + position_y5 + init_y_;
        config().blackboard->set<double>("target_x5", target_x5);
        config().blackboard->set<double>("target_y5", target_y5);
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::ReceiveInitErrorAction>("ReceiveInitError");
}
