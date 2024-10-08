#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class DecisionMakerNode : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    DecisionMakerNode(std::string name) : Node(name)
    {
        tree_name = std::vector<std::string>(2, " ");
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        this->declare_parameter("loop_duration_in_millisec", 10);
        this->get_parameter("loop_duration_in_millisec", loop_duration_in_millisec_);
        bt_loop_duration_ = std::chrono::milliseconds(loop_duration_in_millisec_);
        this->declare_parameter("server_timeout_in_millisec", 100);
        this->get_parameter("server_timeout_in_millisec", server_timeout_in_millisec_);
        this->declare_parameter("wait_for_service_timeout_in_millisec_", 100);
        this->get_parameter("wait_for_service_timeout_in_millisec_", wait_for_service_timeout_in_millisec_);
        wait_for_service_timeout_ = std::chrono::milliseconds(wait_for_service_timeout_in_millisec_);
        server_timeout_ = std::chrono::milliseconds(server_timeout_in_millisec_);
        this->declare_parameter("plugin_lib_names", std::vector<std::string>());
        this->get_parameter("plugin_lib_names", plugin_lib_names_);
        this->declare_parameter("bt_xml_filename", std::string(""));
        this->get_parameter("bt_xml_filename", bt_xml_filename_);
        this->declare_parameter("is_we_are_blue", true);
        this->get_parameter("is_we_are_blue", is_we_are_blue_);
        this->declare_parameter("tree_name", std::vector<std::string>());
        this->get_parameter("tree_name", tree_name);
        waitNav2();
        pose.header.stamp = now();
        pose.header.frame_id = "map";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = -1.0;
        pose.pose.orientation.w = 0.0;
        client_node_name_ = this->get_name();
        auto options = rclcpp::NodeOptions().arguments(
            {"--ros-args",
             "-r",
             std::string("__node:=") +
                 client_node_name_ + "_rclcpp_node",
             "--"});
        client_node_ = std::make_shared<rclcpp::Node>("_", options);
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("robot_bt_decision_maker");
        bt_xml_filename_ = pkg_share_dir + bt_xml_filename_;
        bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);
        tfbuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(), get_node_timers_interface());
        tfbuffer_->setCreateTimerInterface(timer_interface);
        tfbuffer_->setUsingDedicatedThread(true);
        tflistener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_, this, false);
        blackboard_ = BT::Blackboard::create();
        //Put items on the blackboard
        blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);                    // NOLINT
        blackboard_->set<std::chrono::milliseconds>("server_timeout", server_timeout_);     // NOLINT
        blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);
        blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); // NOLINT
        blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tfbuffer_);         // NOLINT
        blackboard_->set<bool>("is_we_are_blue", is_we_are_blue_);
        blackboard_->set<float>("pose_x",0.0);
        blackboard_->set<float>("pose_y",0.0);
        blackboard_->set<bool>("is_adjust",false);
        blackboard_->set<bool>("is_finished",false);
        blackboard_->set<geometry_msgs::msg::PoseStamped>("goal",pose);
        blackboard_->set<bool>("is_goal_reached",false);
        blackboard_->set<std::string>("cmd_status", "avoid");
        blackboard_->set<int>("if_navigation",1);
        blackboard_->set<bool>("is_match_finish", false);
        if(is_we_are_blue_)
        {
            blackboard_->set<float>("exchange_x",-4.11);
            blackboard_->set<float>("exchange_y",-5.0);
        }
        else
        {
            blackboard_->set<float>("exchange_x",19.7);
            blackboard_->set<float>("exchange_y",5.85);
        }
        if (!loadBehaviorTree(bt_xml_filename_, blackboard_))
        {
            RCLCPP_ERROR(this->get_logger(), "加载行为树失败.");
            return;
        }
        // output_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
        // input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_, 10, std::bind(&ObstacleSegmentationNode::cloudCallback, this, std::placeholders::_1));
        tflistener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
    }
    nav2_behavior_tree::BtStatus runBehaviorTree()
    {
        auto is_canceling = [this]() -> bool
        {
            return false;
        };
        auto on_loop = [this]() -> void
        {
            //RCLCPP_INFO(this->get_logger(), "行为树正在运行...");
            rclcpp::spin_some(this->get_node_base_interface());
        };
        // Run the Behavior Tree
        return bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);
    }
    void waitNav2()
    {
        std::string node_service = "/bt_navigator/get_state";
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client = this->create_client<lifecycle_msgs::srv::GetState>(node_service);
        while (!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_INFO(this->get_logger(), "Waiting for bt_navigator to be available");
        }
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        std::string state = "unknown";
        while (state != "active")
        {
            RCLCPP_INFO(this->get_logger(), "等待机器人启动");
            auto future = client->async_send_request(request);
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), future ,std::chrono::seconds(2));
            auto result = future.get(); // 获取结果并存储在变量中
            if (result)
            {
                state = result->current_state.label;
            }
            else{
                RCLCPP_INFO(this->get_logger(), "请求超时");
            }
            rclcpp::Rate(1).sleep();
        }
    }

private:
    // Parameters
    int loop_duration_in_millisec_;
    int server_timeout_in_millisec_;
    int wait_for_service_timeout_in_millisec_;
    std::vector<std::string> plugin_lib_names_;
    std::string bt_xml_filename_;
    bool is_we_are_blue_;

    std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::chrono::milliseconds bt_loop_duration_;
    std::chrono::milliseconds server_timeout_;
    std::chrono::milliseconds wait_for_service_timeout_;
    std::string client_node_name_;
    rclcpp::Node::SharedPtr client_node_;
    std::shared_ptr<tf2_ros::Buffer> tfbuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tflistener_;
    geometry_msgs::msg::PoseStamped pose;
    std::vector<std::string> tree_name;
    bool if_read_tree_finish = false;
    
    bool loadBehaviorTree(const std::string &bt_xml_filename, BT::Blackboard::Ptr blackboard)
    {
        // Read the input BT XML from the specified file into a string
        std::ifstream xml_file(bt_xml_filename);

        if (!xml_file.good())
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
            return false;
        }

        //auto xml_string = std::string(
            //std::istreambuf_iterator<char>(xml_file),
           // std::istreambuf_iterator<char>());

        // Create the Behavior Tree from the XML input
        try
        {
            tree_ = bt_->createTreeFromFile(bt_xml_filename, blackboard);
            for (auto &blackboard : tree_.blackboard_stack)
            {
                blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);                    // NOLINT
                blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout_);     // NOLINT
                blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);
                blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); // NOLINT
                blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tfbuffer_);         // NOLINT
                blackboard->set<bool>("is_we_are_blue", is_we_are_blue_);
                blackboard->set<float>("pose_x",0.0);
                blackboard->set<float>("pose_y",0.0);
                blackboard->set<bool>("is_adjust",false);
                blackboard->set<bool>("is_finished",false);
                blackboard->set<geometry_msgs::msg::PoseStamped>("goal",pose);
                blackboard->set<bool>("is_goal_reached",false);
                blackboard->set<std::string>("cmd_status", "avoid");
                blackboard->set<int>("if_navigation",1);
                blackboard->set<bool>("is_match_finish", false);
                if(is_we_are_blue_)
                {
                    blackboard->set<float>("exchange_x",-4.11);
                    blackboard->set<float>("exchange_y",-5.0);
                }
                else
                {
                    blackboard->set<float>("exchange_x",19.7);
                    blackboard->set<float>("exchange_y",5.85);
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception when loading BT: %s", e.what());
            return false;
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<DecisionMakerNode>("decision_maker_node");
    /* 运行节点，并检测退出信号*/
    // 当节点没有退出时，循环调用runBehaviorTree
    rclcpp::WallRate loop_rate(100);

    while (rclcpp::ok())
    {
        node->runBehaviorTree();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}