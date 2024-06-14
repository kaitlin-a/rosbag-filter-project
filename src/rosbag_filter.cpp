#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

class DataFilter {
public:
    void filterData(const std::string &bag_file) {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        std::vector<std::string> topics = {"/odom", "/scan", "/camera"};
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        rosbag::Bag filtered_bag;
        filtered_bag.open("filtered_" + bag_file, rosbag::bagmode::Write);

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
            }
        }

        bag.close();
        filtered_bag.close();
    }

private:
    bool isVehicleMoving(const nav_msgs::Odometry::ConstPtr &odom) {
        return odom->twist.twist.linear.x != 0.0 || odom->twist.twist.linear.y != 0.0 || odom->twist.twist.linear.z != 0.0 ||
               odom->twist.twist.angular.x != 0.0 || odom->twist.twist.angular.y != 0.0 || odom->twist.twist.angular.z != 0.0;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_filter");
    DataFilter df;
    df.filterData("your_rosbag_file.bag");
    return 0;
}
