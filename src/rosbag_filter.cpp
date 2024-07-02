#include <ros/ros.h>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <filesystem>
#include <vector>
#include <string>
#include <boost/program_options.hpp>
#include <sstream>
#include <map>
#include <set>

namespace fs = std::filesystem;
namespace po = boost::program_options;

void checkTimeRange(const std::string& start, const std::string& end, double bag_start, double bag_end, double& start_t, double& end_t) {
    if (start == "start") {
        start_t = bag_start;
    } else {
        start_t = std::stod(start) + bag_start;
        if (start_t > bag_end) {
            std::cerr << "Please input a valid start time. The bag has duration of " << (bag_end - bag_start) << " seconds." << std::endl;
            exit(1);
        }
    }

    if (end == "end") {
        end_t = bag_end;
    } else {
        end_t = std::stod(end) + bag_start;
        if (end_t < 0 || end_t > bag_end) {
            std::cerr << "Please input a valid end time. The bag has duration of " << (bag_end - bag_start) << " seconds." << std::endl;
            exit(1);
        }
    }
    std::cout << "Input time range valid." << std::endl;
}

bool isVehicleMoving(const nav_msgs::Odometry::ConstPtr& odom, double vehicle_movement_threshold) {
    return std::abs(odom->twist.twist.linear.x) > vehicle_movement_threshold ||
           std::abs(odom->twist.twist.linear.y) > vehicle_movement_threshold ||
           std::abs(odom->twist.twist.linear.z) > vehicle_movement_threshold || 
           std::abs(odom->twist.twist.angular.x) > vehicle_movement_threshold ||
           std::abs(odom->twist.twist.angular.y) > vehicle_movement_threshold ||
           std::abs(odom->twist.twist.angular.z) > vehicle_movement_threshold;
}

void filterData(rosbag::Bag& input_bag, rosbag::Bag& output_bag, const std::set<std::string>& topics, double start_time, double end_time, double vehicle_movement_threshold) {
    rosbag::View view(input_bag, rosbag::TopicQuery(std::vector<std::string>(topics.begin(), topics.end())), ros::Time(start_time), ros::Time(end_time));

    nav_msgs::Odometry::ConstPtr last_odom = nullptr;

    for (const rosbag::MessageInstance &m : view) {
        std::cout << "Processing message from topic: " << m.getTopic() << " at time: " << m.getTime().toSec() << std::endl;

        if (m.getTime().toSec() < start_time || m.getTime().toSec() > end_time) {
            std::cout << "Skipping message outside time range" << std::endl;
            continue;
        }

        if (m.getDataType() == "nav_msgs/Odometry") {
            nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
            if (odom != nullptr) {
                last_odom = odom;
                if (isVehicleMoving(odom, vehicle_movement_threshold)) {
                    std::cout << "Writing /odom message at time: " << m.getTime().toSec() << std::endl;
                    output_bag.write("/odom", m.getTime(), odom);
                }
            }
        } else {
            // For other message types, write directly to the output bag
            std::cout << "Writing message from topic: " << m.getTopic() << " at time: " << m.getTime().toSec() << std::endl;
            output_bag.write(m.getTopic(), m.getTime(), m);
        }
    }

    std::cout << "Filtered bag file created and closed successfully." << std::endl;
}

int main(int argc, char** argv) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("ros_version", po::value<std::string>()->required(), "ROS Version: ros1 or ros2")
        ("start_time", po::value<std::string>()->default_value("start"), "User-defined start time (in seconds)")
        ("end_time", po::value<std::string>()->default_value("end"), "User-defined end time (in seconds)")
        ("bag_file", po::value<std::string>()->required(), "Path to the rosbag file")
        ("topics", po::value<std::string>()->required(), "Topics to include, separated by spaces");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    std::string ros_version = vm["ros_version"].as<std::string>();
    std::string start_time = vm["start_time"].as<std::string>();
    std::string end_time = vm["end_time"].as<std::string>();
    std::string bag_file = vm["bag_file"].as<std::string>();
    std::string input_topics = vm["topics"].as<std::string>();

    // Split topics by spaces
    std::istringstream iss(input_topics);
    std::set<std::string> topics((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());

    // Print topics being filtered
    std::cout << "Topics to filter: ";
    for (const auto& topic : topics) {
        std::cout << topic << " ";
    }
    std::cout << std::endl;

    // Open the bag file to read available topics and get start/end times
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);
    rosbag::View view(bag);
    std::set<std::string> available_topics;
    for (const auto& connection_info : view.getConnections()) {
        available_topics.insert(connection_info->topic);
    }
    double bag_start = view.getBeginTime().toSec();
    double bag_end = view.getEndTime().toSec();
    bag.close();

    // Validate selected topics
    for (const auto& topic : topics) {
        if (available_topics.find(topic) == available_topics.end()) {
            std::cerr << "Topic " << topic << " is not available in the bag file." << std::endl;
            return 1;
        }
    }

    double start_t, end_t;
    checkTimeRange(start_time, end_time, bag_start, bag_end, start_t, end_t);

    // Open the bag file again for reading and create the filtered bag file for writing
    bag.open(bag_file, rosbag::bagmode::Read);
    fs::path original_path(bag_file);
    fs::path filtered_bag_path = original_path.parent_path() / ("filtered_" + original_path.filename().string());

    std::cout << "Creating filtered bag file: " << filtered_bag_path << std::endl;
    rosbag::Bag filtered_bag;
    filtered_bag.open(filtered_bag_path, rosbag::bagmode::Write);

    filterData(bag, filtered_bag, topics, start_t, end_t, 0.01);

    filtered_bag.close();
    bag.close();

    return 0;
}
