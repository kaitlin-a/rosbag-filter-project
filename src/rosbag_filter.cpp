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

class DataFilter {
public:
    DataFilter(const std::set<std::string>& topics,
               ros::Publisher& odom_pub,
               ros::Publisher& scan_pub,
               ros::Publisher& img_pub,
               ros::Publisher& imu_pub,
               ros::Publisher& pc2_pub)
    : topics_(topics), odom_pub_(odom_pub), scan_pub_(scan_pub), img_pub_(img_pub), imu_pub_(imu_pub), pc2_pub_(pc2_pub), last_odom(nullptr) {}

    void filterData(const std::string& bag_file, double start_time, double end_time); 

private:
    bool isVehicleMoving(const nav_msgs::Odometry::ConstPtr& odom);
    bool isValidLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool isValidPointCloud(const sensor_msgs::PointCloud2::ConstPtr& pc2);
    bool isValidImu(const sensor_msgs::Imu::ConstPtr& imu);

    std::set<std::string> topics_;
    nav_msgs::Odometry::ConstPtr last_odom;
    std::string output_bag_file_prefix = "filtered_";
    double vehicle_movement_threshold = 0.01;
    double snr_threshold = 10.0;  // SNR threshold for now, may change later 

    ros::Publisher odom_pub_;
    ros::Publisher scan_pub_;
    ros::Publisher img_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher pc2_pub_;
};

bool DataFilter::isVehicleMoving(const nav_msgs::Odometry::ConstPtr& odom) {
    double threshold = vehicle_movement_threshold;
    return std::abs(odom->twist.twist.linear.x) > threshold ||
           std::abs(odom->twist.twist.linear.y) > threshold ||
           std::abs(odom->twist.twist.linear.z) > threshold || 
           std::abs(odom->twist.twist.angular.x) > threshold ||
           std::abs(odom->twist.twist.angular.y) > threshold ||
           std::abs(odom->twist.twist.angular.z) > threshold;
}

bool DataFilter::isValidLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (scan->intensities[i] < snr_threshold) {
            return false;  // Discard scan if any intensity is below the threshold
        }
    }
    return true;
}

bool DataFilter::isValidPointCloud(const sensor_msgs::PointCloud2::ConstPtr& pc2) {
    sensor_msgs::PointCloud2 pc2_non_const = *pc2;
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(pc2_non_const, "intensity");
    for (; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        if (*iter_intensity < snr_threshold) {
            return false;  // Discard point cloud if any intensity is below the threshold
        }
    }
    return true;
}

bool DataFilter::isValidImu(const sensor_msgs::Imu::ConstPtr& imu) {
    // Simple noise threshold based on standard deviation
    double acc_noise_threshold = 0.05;
    double gyro_noise_threshold = 0.01;

    if (std::abs(imu->linear_acceleration.x) > acc_noise_threshold ||
        std::abs(imu->linear_acceleration.y) > acc_noise_threshold ||
        std::abs(imu->linear_acceleration.z) > acc_noise_threshold ||
        std::abs(imu->angular_velocity.x) > gyro_noise_threshold ||
        std::abs(imu->angular_velocity.y) > gyro_noise_threshold ||
        std::abs(imu->angular_velocity.z) > gyro_noise_threshold) {
        return true;
    }
    return false;  // Discard IMU data if noise is above the threshold
}

void DataFilter::filterData(const std::string& bag_file, double start_time, double end_time) {
    std::cout << "Attempting to open bag file: " << bag_file << std::endl;
    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
        std::cout << "Bag file opened successfully: " << bag_file << std::endl;
    } catch (const rosbag::BagIOException& e) {
        ROS_ERROR("Error opening bag file: %s", bag_file.c_str());
        std::cerr << "Exception: " << e.what() << std::endl;
        return;
    }

    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>(topics_.begin(), topics_.end())));
    fs::path original_path(bag_file);
    fs::path filtered_bag_path = original_path.parent_path() / ("filtered_" + original_path.filename().string());

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
    std::cout << "Processing message from topic: " << m.getTopic() << " at time: " << m.getTime().toSec() << std::endl;

    if (m.getTime().toSec() < start_time || m.getTime().toSec() > end_time) {
        std::cout << "Skipping message outside time range" << std::endl;
        continue;
    }

        if (m.getTopic() == "/odom") {
            nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
            if (odom != nullptr) {                
                last_odom = odom;
                if (isVehicleMoving(odom)) {
                    std::cout << "Writing /odom message at time: " << m.getTime().toSec() << std::endl;
                    filtered_bag.write("/odom", m.getTime(), odom);
                    odom_pub_.publish(odom);
                }
            }
        } else if (m.getTopic() == "/scan") {
            sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
            if (scan != nullptr && last_odom != nullptr && isVehicleMoving(last_odom) && isValidLaserScan(scan)) {
                std::cout << "Writing /scan message at time: " << m.getTime().toSec() << std::endl;
                filtered_bag.write("/scan", m.getTime(), scan);
                scan_pub_.publish(scan);
            }
        } else if (m.getTopic() == "/front_camera/image_raw") {
            sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
            if (img != nullptr && last_odom != nullptr && isVehicleMoving(last_odom)) {
                std::cout << "Writing /front_camera/image_raw message at time: " << m.getTime().toSec() << std::endl;
                filtered_bag.write("/front_camera/image_raw", m.getTime(), img);
                img_pub_.publish(img);
            }
        } else if (m.getTopic() == "/imu/data") {
            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            if (imu != nullptr && last_odom != nullptr && isVehicleMoving(last_odom) && isValidImu(imu)) {
                std::cout << "Writing /imu/data message at time: " << m.getTime().toSec() << std::endl;
                filtered_bag.write("/imu/data", m.getTime(), imu);
                imu_pub_.publish(imu);
            }
        } else if (m.getTopic() == "/lidar_left/velodyne_points" || m.getTopic() == "/lidar_right/velodyne_points") {
            sensor_msgs::PointCloud2::ConstPtr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
            if (pc2 != nullptr && last_odom != nullptr && isVehicleMoving(last_odom) && isValidPointCloud(pc2)) {
                std::cout << "Writing " << m.getTopic() << " message at time: " << m.getTime().toSec() << std::endl;
                filtered_bag.write(m.getTopic(), m.getTime(), pc2);
                pc2_pub_.publish(pc2);
            }
        } else {
        std::cout << "Skipping unsupported topic: " << m.getTopic() << std::endl;
        }
    }

    filtered_bag.close();
    bag.close();
    std::cout << "Filtered bag file created and closed successfully." << std::endl;
}

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_filter");
    ros::NodeHandle nh;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("ros_version", po::value<std::string>()->required(), "ROS Version: ros1 or ros2")
        ("start_time", po::value<std::string>()->default_value("start"), "User-defined start time (in seconds)")
        ("end_time", po::value<std::string>()->default_value("end"), "User-defined end time (in seconds)")
        ("bag_file", po::value<std::string>()->required(), "Path to the rosbag file");

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

    // Open the bag file to read available topics
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);
    rosbag::View view(bag);
    std::set<std::string> available_topics;
    for (const auto& connection_info : view.getConnections()) {
        available_topics.insert(connection_info->topic);
    }
    bag.close();

    // Print available topics, ask user for selection
    std::cout << "Available topics in the bag file:" << std::endl;
    for (const auto& topic : available_topics) {
        std::cout << topic << std::endl;
    }

    std::cout << "Please enter the topics provided above that you want to include, separated by spaces (e.g., /odom /scan /imu/data): ";
    std::string input_topics;
    std::getline(std::cin, input_topics);

    std::istringstream iss(input_topics);
    std::set<std::string> topics((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());

    // Ensure that we include essential topics by default
    std::set<std::string> essential_topics = {
        "/odom", 
        "/scan", 
        "/front_camera/image_raw",
        "/imu/data", 
        "/lidar_left/velodyne_points", 
        "/lidar_right/velodyne_points"
        };
    for (const auto& topic : essential_topics) {
        if (available_topics.find(topic) != available_topics.end()) {
            topics.insert(topic);
        }
    }

    // Validate selected topics
    for (const auto& topic : topics) {
        if (available_topics.find(topic) == available_topics.end()) {
            std::cerr << "Topic " << topic << " is not available in the bag file." << std::endl;
            return 1;
        }
    }

    // Open the bag file again to read the start and end time
    bag.open(bag_file, rosbag::bagmode::Read);
    rosbag::View view_full(bag);
    double bag_start = view_full.getBeginTime().toSec();
    double bag_end = view_full.getEndTime().toSec();
    bag.close();

    double start_t, end_t;
    checkTimeRange(start_time, end_time, bag_start, bag_end, start_t, end_t);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/filtered/odom", 10);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/filtered/scan", 10);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/filtered/image", 10);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/filtered/imu", 10);
    ros::Publisher pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered/pointcloud2", 10);

    DataFilter filter(topics, odom_pub, scan_pub, img_pub, imu_pub, pc2_pub);
    filter.filterData(bag_file, start_t, end_t);

    return 0;
}
