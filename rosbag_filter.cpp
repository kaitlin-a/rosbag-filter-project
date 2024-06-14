#include <ros/ros.h>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

class DataFilter {
public:
    void filterData(const std::string& bag_file) {
        std::cout << "Attempting to open bag file: " << bag_file << std::endl;
        rosbag::Bag bag;
        try {
            bag.open(bag_file, rosbag::bagmode::Read);
            std::cout << "Bag file opened successfully." << bag_file << std::endl;
        } catch (const rosbag::BagIOException& e) {
            ROS_ERROR("Error opening bag file: %s", bag_file.c_str());
            std::cerr << "Exception: " << e.what() << std::endl;
        return;
    }

        std::vector<std::string> topics = 
        {"/odom", "/scan", "/camera", "/imu/data"};
        rosbag::View view(bag, rosbag::TopicQuery(topics));


        std::string filtered_bag_path = "filtered_" + bag_file;
        std::cout << "Creating filtered bag file: " << filtered_bag_path << std::endl;
        rosbag::Bag filtered_bag;
        try {
        filtered_bag.open(filtered_bag_path, rosbag::bagmode::Write);
        std::cout << "Filtered bag file opened successfully: " << filtered_bag_path << std::endl;
        } catch (const rosbag::BagIOException& e) {
            ROS_ERROR("Error opening filtered bag file: %s", filtered_bag_path.c_str());
            std::cerr << "Exception: " << e.what() << std::endl;
            return;
        }

        for (const rosbag::MessageInstance &m : view) {
            if (m.getTopic() == "/odom") {
                nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();

                if (odom != nullptr && isVehicleMoving(odom)) {
                    filtered_bag.write("/odom", m.getTime(), odom);
                }

            } else if (m.getTopic() == "/scan") {
                sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
                if (scan != nullptr) {
                    filtered_bag.write("/scan", m.getTime(), scan);
                }

            } else if (m.getTopic() == "/camera") {
                sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
                if (img != nullptr) {
                    filtered_bag.write("/camera", m.getTime(), img);
                }
            
            } else if (m.getTopic() == "/imu/data") {
                sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
                if (imu != nullptr) {
                    filtered_bag.write("/imu/data", m.getTime(), imu);
                }
            }
        }

        filtered_bag.close();
        bag.close();
        std::cout << "Filtered bag file created and closed successfully." << std::endl;
    }

private:
    bool isVehicleMoving(const nav_msgs::Odometry::ConstPtr& odom) {
        return odom->twist.twist.linear.x != 0.0 || 
               odom->twist.twist.linear.y != 0.0 || 
               odom->twist.twist.linear.z != 0.0 ||

               odom->twist.twist.angular.x != 0.0 || 
               odom->twist.twist.angular.y != 0.0 || 
               odom->twist.twist.angular.z != 0.0;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_filter");
    if (argc != 2) {
        ROS_ERROR("Usage: rosbag_filter <bag_file>");
        return 1;
    }

    std::string bag_file = argv[1];
    std::cout << "Bag file path: " << bag_file << std::endl;
    DataFilter filter;
    filter.filterData(bag_file);
    
    return 0;
}
