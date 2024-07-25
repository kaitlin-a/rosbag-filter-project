#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <boost/program_options.hpp>
#include <filesystem>
#include <iostream>
#include <string>
#include <cstdlib>
#include <set>
#include <sstream>
#include <limits>

namespace fs = std::filesystem;
namespace po = boost::program_options;

class KalmanFilter {
public:
    KalmanFilter() {
        // State vector
        x_ = Eigen::VectorXd(6);
        x_.setZero();

        // State covariance matrix
        P_ = Eigen::MatrixXd(6, 6);
        P_.setIdentity();
        P_ *= 0.1;

        // State transition matrix, constant velocity model
        F_ = Eigen::MatrixXd(6, 6);
        double dt = 0.1; // Time step
        F_ << 1, 0, 0, dt, 0, 0,
              0, 1, 0, 0, dt, 0,
              0, 0, 1, 0, 0, dt,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1;

        // Process noise covariance matrix
        Q_ = Eigen::MatrixXd(6, 6);
        Q_.setIdentity();
        Q_ *= 0.1; // TBD, double check expected process noise

        // Measurement matrix
        H_ = Eigen::MatrixXd(6, 6);
        H_.setIdentity();

        // Measurement noise covariance matrix
        R_ = Eigen::MatrixXd(3, 3);
        R_.setIdentity();
        R_ *= 0.1; // TBD, double check expected measurement noise

        initialized_ = false;
    }

    void predict() {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_; // Prediction step
    }

    void update(const Eigen::VectorXd& z) { // Update step
        if (!initialized_) {
            x_.head(3) = z.head(3); // Initialize the position part of the state vector
            initialized_ = true;
            return;
        }

        Eigen::VectorXd y = z - H_ * x_; // Residual
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; // Innovation covariance
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); // Kalman gain

        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_; // Update state estimate & covariance
    }

    double calculateSNR(const Eigen::VectorXd& signal, const Eigen::VectorXd& noise) {
        double signal_power = signal.squaredNorm() / signal.size();
        double noise_power = noise.squaredNorm() / noise.size();
        return 10 * log10(signal_power / noise_power);
    }

    void adjustMeasurementNoise(const Eigen::VectorXd& z) {
        Eigen::VectorXd noise = z - H_ * x_;
        double snr = calculateSNR(z, noise);
        double scaling_factor = (snr < 10) ? 10 : (snr > 30) ? 0.1 : 1.0 / snr;
        R_ *= scaling_factor;
    }

    Eigen::VectorXd getState() const {
        return x_;
    }

    bool isMoving(double threshold = 0.1) const {
        return x_.tail(3).norm() > threshold;
    }

private:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_, F_, Q_, H_, R_;
    bool initialized_;
};

void filterOdometry(const nav_msgs::Odometry::ConstPtr& odom_msg, KalmanFilter& kf) {
    // Convert odometry data to Eigen vector for updating the Kalman filter
    Eigen::VectorXd z(3);
    z << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;
    kf.adjustMeasurementNoise(z); // Adjust measurement noise based on SNR
    kf.update(z);
}

void filterData(rosbag::Bag& input_bag, rosbag::Bag& output_bag, const std::set<std::string>& topics, double start_time, double end_time, KalmanFilter& kf) {
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
                Eigen::VectorXd z(6);
                z << odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z,
                     odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;
                kf.adjustMeasurementNoise(z); // Adjust measurement noise based on SNR
                kf.update(z);
                if (kf.isMoving()) {
                    std::cout << "Writing /odom message at time: " << m.getTime().toSec() << std::endl;
                    output_bag.write(m.getTopic(), m.getTime(), odom);
                }
                last_odom = odom;
            }
        } else if (m.getDataType() == "sensor_msgs/Imu") {
            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            if (imu != nullptr && last_odom != nullptr) {
                std::cout << "Writing /imu/data message at time: " << m.getTime().toSec() << std::endl;
                output_bag.write(m.getTopic(), m.getTime(), imu);
            }
        } else {
            std::cout << "Writing unsupported topic: " << m.getTopic() << std::endl;
            output_bag.write(m.getTopic(), m.getTime(), m);
        }
    }

    std::cout << "Filtered bag file created and closed successfully." << std::endl;
}

void processBagFile(const std::string& bag_file, const std::set<std::string>& topics, double start_t, double end_t, KalmanFilter& kf) {
    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return;
    }

    rosbag::View view(bag);
    double bag_start = view.getBeginTime().toSec();
    double bag_end = view.getEndTime().toSec();
    bag.close();

    // Print debug information
    std::cout << "Bag start time: " << bag_start << ", Bag end time: " << bag_end << std::endl;
    std::cout << "Requested start time: " << start_t << ", Requested end time: " << end_t << std::endl;

    // Check and adjust time range
    if (start_t == 0) start_t = bag_start;
    if (end_t == std::numeric_limits<double>::max()) end_t = bag_end;

    std::cout << "Adjusted start time: " << start_t << ", Adjusted end time: " << end_t << std::endl;

    std::cout << "Processing bag: " << bag_file << std::endl;

    bag.open(bag_file, rosbag::bagmode::Read);
    fs::path original_path(bag_file);
    fs::path filtered_bag_path = original_path.parent_path() / ("filtered_" + original_path.filename().string());

    rosbag::Bag filtered_bag;
    filtered_bag.open(filtered_bag_path, rosbag::bagmode::Write);

    filterData(bag, filtered_bag, topics, start_t, end_t, kf);

    filtered_bag.close();
    bag.close();
}

int main(int argc, char** argv) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("ros_version", po::value<std::string>()->required(), "ROS Version: ros1 or ros2")
        ("start_time", po::value<std::string>()->default_value("start"), "User-defined start time (in seconds)")
        ("end_time", po::value<std::string>()->default_value("end"), "User-defined end time (in seconds)")
        ("bag_file", po::value<std::string>(), "Path to the rosbag file")
        ("bag_folder", po::value<std::string>(), "Path to the folder containing rosbag files")
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
    std::string input_topics = vm["topics"].as<std::string>();

    std::set<std::string> topics;
    std::istringstream iss(input_topics);
    for (std::string s; iss >> s;) {
        topics.insert(s);
    }

    double start_t = (start_time == "start") ? 0 : std::stod(start_time);
    double end_t = (end_time == "end") ? std::numeric_limits<double>::max() : std::stod(end_time);

    KalmanFilter kf;

    if (vm.count("bag_file")) {
        std::string bag_file = vm["bag_file"].as<std::string>();
        processBagFile(bag_file, topics, start_t, end_t, kf);
    } else if (vm.count("bag_folder")) {
        std::string bag_folder = vm["bag_folder"].as<std::string>();
        for (const auto& entry : fs::directory_iterator(bag_folder)) {
            if (entry.path().extension() == ".bag") {
                processBagFile(entry.path().string(), topics, start_t, end_t, kf);
            }
        }
    } else {
        std::cerr << "Please specify either --bag_file or --bag_folder." << std::endl;
        return 1;
    }

    return 0;
}
