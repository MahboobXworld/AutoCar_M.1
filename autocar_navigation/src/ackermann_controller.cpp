#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <algorithm>
#include <vector>

class AckermannController {
public:
    AckermannController() : 
        x_(0.0), y_(0.0), theta_(0.0), 
        last_time_initialized_(false)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Load Parameters
        private_nh.param<double>("wheelbase", L_, 0.70);
        private_nh.param<double>("track_width", W_, 0.80);
        private_nh.param<double>("wheel_radius", R_, 0.12);
        private_nh.param<double>("max_front_wheel_angle_rad", max_steer_rad_, 0.785398); // 45 degrees

        // Publishers for Joint Controllers
        fl_steer_pub_ = nh.advertise<std_msgs::Float64>("/front_left_steering_position_controller/command", 1);
        fr_steer_pub_ = nh.advertise<std_msgs::Float64>("/front_right_steering_position_controller/command", 1);
        
        fl_wheel_pub_ = nh.advertise<std_msgs::Float64>("/front_left_wheel_velocity_controller/command", 1);
        fr_wheel_pub_ = nh.advertise<std_msgs::Float64>("/front_right_wheel_velocity_controller/command", 1);
        rl_wheel_pub_ = nh.advertise<std_msgs::Float64>("/rear_left_wheel_velocity_controller/command", 1);
        rr_wheel_pub_ = nh.advertise<std_msgs::Float64>("/rear_right_wheel_velocity_controller/command", 1);

        // Odometry Publisher
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/diff_drive_controller/odom", 5);

        // Subscribers
        cmd_sub_ = nh.subscribe("/cmd_vel", 1, &AckermannController::cmdVelCallback, this);
        joint_sub_ = nh.subscribe("/joint_states", 5, &AckermannController::jointStateCallback, this);

        ROS_INFO("C++ AckermannController initialized. Wheelbase: %.2fm, Track: %.2fm, Max Steer Rad: %.3f", 
                 L_, W_, max_steer_rad_);
    }

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        double v = msg->linear.x; // Target linear speed in m/s
        double w = msg->angular.z; // Target yaw rate in rad/s

        ROS_INFO_THROTTLE(1.0, "Received cmd_vel: v = %.3f m/s, w = %.3f rad/s", v, w);

        double delta = 0.0;
        if (std::abs(v) < 0.001) {
            v = 0.0;
            // Standstill steering
            if (std::abs(w) > 0.01) {
                delta = (w > 0) ? max_steer_rad_ : -max_steer_rad_;
            }
        } else {
            // Ackerman steering geometry: delta = atan2(w * L, abs(v))
            delta = std::atan2(w * L_, std::abs(v));
            delta = std::max(-max_steer_rad_, std::min(max_steer_rad_, delta));

            // Apply direction of velocity to steering direction
            if (v < 0.0) {
                delta = -delta;
            }
        }

        double delta_l = 0.0;
        double delta_r = 0.0;
        double v_rl = v;
        double v_rr = v;
        double v_fl = v;
        double v_fr = v;

        if (std::abs(delta) > 0.001) {
            double tan_delta = std::tan(delta);
            
            // Ackerman steering geometry formulas
            delta_l = std::atan(L_ / (L_ / tan_delta - W_ / 2.0));
            delta_r = std::atan(L_ / (L_ / tan_delta + W_ / 2.0));

            // Clamp left/right wheel steer angles
            delta_l = std::max(-max_steer_rad_, std::min(max_steer_rad_, delta_l));
            delta_r = std::max(-max_steer_rad_, std::min(max_steer_rad_, delta_r));

            // Calculate inside/outside rear wheel linear velocities
            double radius_ratio_l = 1.0 - (W_ / (2.0 * L_)) * tan_delta;
            double radius_ratio_r = 1.0 + (W_ / (2.0 * L_)) * tan_delta;
            
            v_rl = v * radius_ratio_l;
            v_rr = v * radius_ratio_r;

            // Calculate front wheel velocities matching the steer angle
            v_fl = v_rl / std::cos(delta_l);
            v_fr = v_rr / std::cos(delta_r);
        }

        // Left wheels (fl, rl) require positive command to rotate CCW (forward motion)
        // Right wheels (fr, rr) require positive command to rotate CCW (forward motion)
        double w_fl = v_fl / R_;
        double w_rl = v_rl / R_;
        double w_fr = v_fr / R_;
        double w_rr = v_rr / R_;

        // Publish to topics
        std_msgs::Float64 msg_fl_steer, msg_fr_steer;
        msg_fl_steer.data = delta_l;
        msg_fr_steer.data = delta_r;
        fl_steer_pub_.publish(msg_fl_steer);
        fr_steer_pub_.publish(msg_fr_steer);

        std_msgs::Float64 msg_fl_wheel, msg_fr_wheel, msg_rl_wheel, msg_rr_wheel;
        msg_fl_wheel.data = w_fl;
        msg_fr_wheel.data = w_fr;
        msg_rl_wheel.data = w_rl;
        msg_rr_wheel.data = w_rr;
        fl_wheel_pub_.publish(msg_fl_wheel);
        fr_wheel_pub_.publish(msg_fr_wheel);
        rl_wheel_pub_.publish(msg_rl_wheel);
        rr_wheel_pub_.publish(msg_rr_wheel);

        ROS_INFO_THROTTLE(1.0, "Published commands: steer_l=%.3f, steer_r=%.3f, w_fl=%.3f, w_fr=%.3f, w_rl=%.3f, w_rr=%.3f",
                          delta_l, delta_r, w_fl, w_fr, w_rl, w_rr);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        int rl_idx = -1;
        int rr_idx = -1;
        int fl_s_idx = -1;
        int fr_s_idx = -1;

        // Locate joint indexes dynamically by name
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "rear_left_wheel_joint") rl_idx = i;
            else if (msg->name[i] == "rear_right_wheel_joint") rr_idx = i;
            else if (msg->name[i] == "front_left_steering_joint") fl_s_idx = i;
            else if (msg->name[i] == "front_right_steering_joint") fr_s_idx = i;
        }

        // Return if joints are not fully populated yet
        if (rl_idx == -1 || rr_idx == -1 || fl_s_idx == -1 || fr_s_idx == -1) {
            return;
        }

        ros::Time current_time = msg->header.stamp;
        if (current_time.toSec() == 0.0) {
            current_time = ros::Time::now();
        }

        if (!last_time_initialized_) {
            last_time_ = current_time;
            last_time_initialized_ = true;
            return;
        }

        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        if (dt <= 0.0 || dt > 0.5) {
            return;
        }

        // Get joint states
        double rl_vel = msg->velocity[rl_idx];      // rad/s
        double rr_vel = msg->velocity[rr_idx];      // rad/s
        double fl_steer = msg->position[fl_s_idx];  // rad
        double fr_steer = msg->position[fr_s_idx];  // rad

        // Rear wheel linear velocity (Both sides positive roll forward)
        double v_rl = rl_vel * R_;
        double v_rr = rr_vel * R_;
        double v_x = (v_rl + v_rr) / 2.0;

        // Average steering angle
        double delta = (fl_steer + fr_steer) / 2.0;

        // Ackerman angular velocity
        double w_z = (v_x * std::tan(delta)) / L_;

        // Integrate pose
        theta_ += w_z * dt;
        x_ += v_x * std::cos(theta_) * dt;
        y_ += v_x * std::sin(theta_) * dt;

        // Build and publish Odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = w_z;

        odom.pose.covariance = {
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        };
        odom.twist.covariance = {
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        };

        odom_pub_.publish(odom);
    }

    // Parameters
    double L_;                  // Wheelbase
    double W_;                  // Track width
    double R_;                  // Wheel radius
    double max_steer_rad_;      // Max physical wheel angle in rad (default 45 degrees)

    // Odometry state
    double x_, y_, theta_;
    ros::Time last_time_;
    bool last_time_initialized_;

    // Publishers & Subscribers
    ros::Publisher fl_steer_pub_, fr_steer_pub_;
    ros::Publisher fl_wheel_pub_, fr_wheel_pub_, rl_wheel_pub_, rr_wheel_pub_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber joint_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_controller");
    AckermannController controller;
    ros::spin();
    return 0;
}
