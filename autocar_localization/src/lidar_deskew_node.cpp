#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

class LidarDeskewNode {
public:
    LidarDeskewNode() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Cache velocities
        v_x_ = 0.0;
        w_z_ = 0.0;

        // Subscribers
        sub_odom_ = nh.subscribe("/odometry/filtered", 1, &LidarDeskewNode::odomCallback, this);
        sub_scan_ = nh.subscribe("/scan", 5, &LidarDeskewNode::scanCallback, this);

        // Publisher
        pub_scan_ = nh.advertise<sensor_msgs::LaserScan>("/scan_deskewed", 5);

        ROS_INFO("[LidarDeskewNode] Initialized C++ node and ready to deskew scans.");
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        v_x_ = msg->twist.twist.linear.x;
        w_z_ = msg->twist.twist.angular.z;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        double v = v_x_;
        double w = w_z_;

        // If robot is practically stationary, skip compensation to save CPU
        if (std::abs(v) < 0.01 && std::abs(w) < 0.01) {
            pub_scan_.publish(msg);
            return;
        }

        // Prepare new laser scan message
        sensor_msgs::LaserScan new_scan;
        new_scan.header = msg->header;
        new_scan.angle_min = msg->angle_min;
        new_scan.angle_max = msg->angle_max;
        new_scan.angle_increment = msg->angle_increment;
        new_scan.time_increment = msg->time_increment;
        new_scan.scan_time = msg->scan_time;
        new_scan.range_min = msg->range_min;
        new_scan.range_max = msg->range_max;

        size_t n_beams = msg->ranges.size();
        new_scan.ranges.assign(n_beams, std::numeric_limits<float>::infinity());

        double time_increment = msg->time_increment;
        if (time_increment == 0.0) {
            time_increment = msg->scan_time / static_cast<double>(n_beams);
        }

        for (size_t i = 0; i < n_beams; ++i) {
            float r = msg->ranges[i];
            // Filter invalid ranges
            if (std::isnan(r) || std::isinf(r) || r < msg->range_min || r > msg->range_max) {
                continue;
            }

            double t = i * time_increment;
            double delta_theta = w * t;
            double delta_x = 0.0;
            double delta_y = 0.0;

            if (std::abs(w) > 1e-5) {
                delta_x = (v / w) * std::sin(delta_theta);
                delta_y = (v / w) * (1.0 - std::cos(delta_theta));
            } else {
                delta_x = v * t;
                delta_y = 0.0;
            }

            // Project point in measurement frame (polar to cartesian)
            double angle = msg->angle_min + i * msg->angle_increment;
            double px = r * std::cos(angle);
            double py = r * std::sin(angle);

            // Transform points back to the robot starting frame (t=0)
            double px_deskewed = delta_x + px * std::cos(delta_theta) - py * std::sin(delta_theta);
            double py_deskewed = delta_y + px * std::sin(delta_theta) + py * std::cos(delta_theta);

            // Convert back to polar coordinates
            float r_deskewed = std::sqrt(px_deskewed * px_deskewed + py_deskewed * py_deskewed);
            double angle_deskewed = std::atan2(py_deskewed, px_deskewed);

            // Map to nearest nominal beam index
            int bin_idx = std::round((angle_deskewed - msg->angle_min) / msg->angle_increment);

            // Filter indices inside range
            if (bin_idx >= 0 && bin_idx < static_cast<int>(n_beams)) {
                new_scan.ranges[bin_idx] = std::min(new_scan.ranges[bin_idx], r_deskewed);
            }
        }

        pub_scan_.publish(new_scan);
    }

    double v_x_;
    double w_z_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_scan_;
    ros::Publisher pub_scan_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_deskew_node");
    LidarDeskewNode node;
    ros::spin();
    return 0;
}
