#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node;
    node = std::make_shared<rclcpp::Node>("merge_pcd_node");
    RCLCPP_INFO(node->get_logger(), "merge_pcd_node start");

    std::string pcd_folder_;
    node->declare_parameter("pcd_folder", "/home/sentry111/licang_ws/src/licang/point_lio_ros2/point_lio/PCD");
    node->get_parameter("pcd_folder", pcd_folder_);

    // 获取该路径下的所有文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto &file : std::filesystem::directory_iterator(pcd_folder_))
    {
        if (file.path().extension() == ".pcd")
        {
            RCLCPP_INFO(node->get_logger(), "Loading %s", file.path().c_str());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(file.path(), *cloud_temp) == -1)
            {
                PCL_ERROR("Couldn't read file %s\n", file.path());
                return -1;
            }
            *cloud += *cloud_temp;
        }
    }
    // 保存合并后的pcd文件
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char merge_file_name[200];
    sprintf(merge_file_name, "/%d_%02d_%02d_%02d_%02d_%02d_merged.pcd", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    std::string merge_file_name_string(merge_file_name);
    pcl::io::savePCDFileASCII(pcd_folder_ + merge_file_name_string, *cloud);
    RCLCPP_INFO(node->get_logger(), "Saved %s", (pcd_folder_ + merge_file_name_string).c_str());


    return 0;
}
