#include <ros/ros.h>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <filesystem>
#include <vector> 
#include <string> 
#include <boost/program_options.hpp>

namespace fs = std::filesystem;
namespace po = boost::program_options;

class DataFilter {
public:
    DataFilter(const std::vector<std::string>& topics) : topics_(topics) {}

    void filterData(const std::string& bag_file) {

        std::cout << "Attempting to open bag file: " << bag_file << std::endl;
        rosbag::Bag bag;
        try {
            bag.open(bag_file, rosbag::bagmode::Read);
            std::cout << "Bag file opened successfully." << bag_file << std::endl;
     }  catch (const rosbag::BagIOException& e) {
            ROS_ERROR("Error opening bag file: %s", bag_file.c_str());
            std::cerr << "Exception: " << e.what() << std::endl;
        return;
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics_));
    fs::path original_path(bag_file);
    fs::path filtered_bag_path = original_path.parent_path() / ("filtered_" + orginal_path.filename().string());

    std::cout << "Creating filtered bag file: " << filtered_bag_path << std::endl;
    rosbag::Bag filtered_bag;
        try {
            filtered_bag.open(filtered_bag_path, rosbag::bagmode::Write);
            std::cout << "Filtered bag file opened successfully: " << filtered_bag_path << std::endl;
    }   catch (const rosbag::BagIOException& e) {
            ROS_ERROR("Error opening filtered bag file: %s", filtered_bag_path.c_str());
            std::cerr << "Exception: " << e.what() << std::endl;
        return;
    }

    for (const rosbag::MessageInstance &m : view) {
        if (m.getTopic() == "/odom") {
            nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();

            if (odom != nullptr) {                
                    last_odom = odom;
                    if (isVehicleMoving(odom)) {
                        filtered_bag.write("/odom", m.getTime(), odom);
                    }
            }

        } else if (m.getTopic() == "/scan") {
                sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
                if (scan != nullptr && last_odom != nullptr && isVehicleMoving(last_odom)) {
                    filtered_bag.write("/scan", m.getTime(), scan);
                }

            } else if (m.getTopic() == "/camera") {
                sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
                if (img != nullptr && last_odom != nullptr && isVehicleMoving(last_odom)) {
                    filtered_bag.write("/camera", m.getTime(), img);
                }
            
            } else if (m.getTopic() == "/imu/data") {
                sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
                if (imu != nullptr && last_odom != nullptr && isVehicleMoving(last_odom)) {
                    filtered_bag.write("/imu/data", m.getTime(), imu);
                }

            } else if (m.getTopic() == "/lidar_left/velodyne_points" || m.getTopic()
                        == "/lidar_right/velodyne_points") {
                            sensor_msgs::PointCloud2::ConstPtr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
                            if (pc2 != nullptr && last_odom != nullptr && isVehicleMoving(last_odom)) {
                                filtered_bag.write(m.getTopic(), m.getTime(), pc2);
                            }
                }  
            
        }

        filtered_bag.close();
        bag.close();
        std::cout << "Filtered bag file created and closed successfully." << std::endl;
    }

private:

    bool isVehicleMoving(const nav_msgs::Odometry::ConstPtr& odom) {
        double threshold = 0.01; //Possible adjustment later ? 
        return  std::abs(odom->twist.twist.linear.x) > threshold ||
                std::abs(odom->twist.twist.linear.y) > threshold ||
                std::abs(odom->twist.twist.linear.z) > threshold || 
                std::abs(odom->twist.twist.angular.x) > threshold ||
                std::abs(odom->twist.twist.angular.y) > threshold ||
                std::abs(odom->twist.twist.angular.z) > threshold;
    }

    std::vector<std::string> topics_;
    nav_msgs::Odometry::ConstPtr last_odom;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_filter");
    if (argc < 2) {
        ROS_ERROR("Usage: rosbag_filter <bag_file> [--topics topic1 topic2 ...]");
        return 1;
    }

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("bag_file", po::value<std::string>(), "path to the rosbag file")
        ("topics", po::value<std::vector<std::string>>()->multitoken(), "list of topics to filters");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;

        return 1;
    }

    std::string bag_file;
    if (vm.count("bag_file")) {
        bag_file = vm["bag_file"].as<std::string>();
    }  else {
        ROS_ERROR("Bag file not specified");
        return 1;
    
    }

    std::vector<std::string> topics;
    if (vm.count("topics")) {
        topics = vm["topics"].as<std::vector<std::string>>();
    } else {
        topics = {"/odom", "/scan", "/camera", "/imu/data"};
    }
    
    DataFilter filter;
    filter.filterData(bag_file);
    
    return 0;
}
