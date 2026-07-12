#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <controller_manager_msgs/SwitchController.h>
#include <tf2/utils.h>
#include <cmath>
#include <string>
#include <vector>

class GoalStopManager {
public:
    GoalStopManager() :
        has_goal_(false),
        in_hold_state_(false),
        controllers_enabled_(true),
        goal_x_(0.0), goal_y_(0.0), goal_yaw_(0.0),
        robot_x_(0.0), robot_y_(0.0), robot_yaw_(0.0),
        robot_v_(0.0),
        hold_x_(0.0), hold_y_(0.0), hold_yaw_(0.0)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Load parameters
        private_nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.20); // meters
        private_nh.param<double>("goal_yaw_tolerance", goal_yaw_tolerance_, 0.15);   // radians (approx 8.6 degrees)
        private_nh.param<double>("vel_stop_threshold", vel_stop_threshold_, 0.05);   // m/s
        private_nh.param<double>("drift_tolerance", drift_tolerance_, 0.25);         // meters
        private_nh.param<double>("hold_delay", hold_delay_, 1.0);                   // seconds before disabling controllers

        // Subscribers
        sub_goal_ = nh.subscribe("/move_base/current_goal", 1, &GoalStopManager::goalCallback, this);
        sub_odom_ = nh.subscribe("/odometry/filtered", 1, &GoalStopManager::odomCallback, this);
        sub_cmd_vel_ = nh.subscribe("/cmd_vel", 1, &GoalStopManager::cmdVelCallback, this);

        // Publishers
        pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_to_controller", 1);
        pub_hold_state_ = nh.advertise<std_msgs::String>("/goal_hold_state", 1, true);

        // Service Client
        client_switch_controller_ = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

        wheel_controllers_ = {
            "front_left_wheel_velocity_controller",
            "front_right_wheel_velocity_controller",
            "rear_left_wheel_velocity_controller",
            "rear_right_wheel_velocity_controller"
        };

        publishState("INIT");
        ROS_INFO("[GoalStopManager] Initialized. Dist Tol: %.2fm, Yaw Tol: %.2frad, Vel Thresh: %.3fm/s, Drift Tol: %.2fm",
                 goal_dist_tolerance_, goal_yaw_tolerance_, vel_stop_threshold_, drift_tolerance_);
    }

    ~GoalStopManager() {
        // Safe exit: make sure controllers are enabled
        setControllersEnabled(true);
    }

    void spin() {
        ros::Rate rate(20); // 20Hz update loop
        while (ros::ok()) {
            ros::spinOnce();
            checkHoldState();
            rate.sleep();
        }
    }

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double new_x = msg->pose.position.x;
        double new_y = msg->pose.position.y;
        double new_yaw = tf2::getYaw(msg->pose.orientation);

        // Check if it is a new goal
        double goal_diff = std::sqrt(std::pow(new_x - goal_x_, 2) + std::pow(new_y - goal_y_, 2));
        if (!has_goal_ || goal_diff > 0.1 || std::abs(new_yaw - goal_yaw_) > 0.1) {
            ROS_INFO("[GoalStopManager] Received new target goal: (%.2f, %.2f, %.2f)", new_x, new_y, new_yaw);
            goal_x_ = new_x;
            goal_y_ = new_y;
            goal_yaw_ = new_yaw;
            has_goal_ = true;

            if (in_hold_state_) {
                ROS_INFO("[GoalStopManager] Exiting Hold State because a new goal was received.");
                in_hold_state_ = false;
                setControllersEnabled(true);
                publishState("NAVIGATING");
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        robot_v_ = msg->twist.twist.linear.x;
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        if (in_hold_state_) {
            // In hold state, ignore external planner commands and publish exactly 0.0
            geometry_msgs::Twist hold_cmd;
            hold_cmd.linear.x = 0.0;
            hold_cmd.angular.z = 0.0;
            pub_cmd_vel_.publish(hold_cmd);
        } else {
            // Pass-through normal commands
            pub_cmd_vel_.publish(msg);
        }
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void checkHoldState() {
        if (!has_goal_) return;

        double dx = goal_x_ - robot_x_;
        double dy = goal_y_ - robot_y_;
        double dist = std::sqrt(dx*dx + dy*dy);
        double heading_err = std::abs(normalizeAngle(goal_yaw_ - robot_yaw_));

        if (!in_hold_state_) {
            // Check if we reached the goal and can enter Hold State
            if (dist < goal_dist_tolerance_ && heading_err < goal_yaw_tolerance_ && std::abs(robot_v_) < vel_stop_threshold_) {
                in_hold_state_ = true;
                hold_x_ = robot_x_;
                hold_y_ = robot_y_;
                hold_yaw_ = robot_yaw_;
                hold_start_time_ = ros::Time::now();
                ROS_INFO("[GoalStopManager] Goal reached! Entering Hold State. Goal: (%.2f, %.2f, %.2f). Hold Pose: (%.2f, %.2f, %.2f)",
                         goal_x_, goal_y_, goal_yaw_, hold_x_, hold_y_, hold_yaw_);
                publishState("HOLDING");
            }
        } else {
            // Monitor drift
            double drift_dist = std::sqrt(std::pow(robot_x_ - hold_x_, 2) + std::pow(robot_y_ - hold_y_, 2));
            if (drift_dist > drift_tolerance_) {
                ROS_WARN("[GoalStopManager] Vehicle drifted %.2fm (limit %.2fm). Exiting Hold State to allow replanning.",
                         drift_dist, drift_tolerance_);
                in_hold_state_ = false;
                setControllersEnabled(true);
                publishState("NAVIGATING");
                return;
            }

            // Gradually center steering (already achieved by publishing v=0, w=0)
            // Disable wheel controllers after a safety delay to allow the vehicle to settle
            if (controllers_enabled_ && (ros::Time::now() - hold_start_time_).toSec() > hold_delay_) {
                setControllersEnabled(false);
            }
        }
    }

    void setControllersEnabled(bool enable) {
        if (controllers_enabled_ == enable) return;

        controller_manager_msgs::SwitchController srv;
        srv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        
        if (enable) {
            srv.request.start_controllers = wheel_controllers_;
            ROS_INFO("[GoalStopManager] Requesting startup of wheel velocity controllers.");
        } else {
            srv.request.stop_controllers = wheel_controllers_;
            ROS_INFO("[GoalStopManager] Requesting shutdown of wheel velocity controllers (allowing passive damping).");
        }

        if (client_switch_controller_.call(srv)) {
            if (srv.response.ok) {
                controllers_enabled_ = enable;
                ROS_INFO("[GoalStopManager] Successfully switched wheel controllers.");
            } else {
                ROS_WARN("[GoalStopManager] Controller manager rejected the switch request.");
            }
        } else {
            ROS_WARN("[GoalStopManager] Switch controller service not available.");
        }
    }

    void publishState(const std::string& state) {
        std_msgs::String msg;
        msg.data = state;
        pub_hold_state_.publish(msg);
    }

    // Parameters
    double goal_dist_tolerance_;
    double goal_yaw_tolerance_;
    double vel_stop_threshold_;
    double drift_tolerance_;
    double hold_delay_;

    // Subscribers & Publishers
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_cmd_vel_;
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_hold_state_;
    ros::ServiceClient client_switch_controller_;

    // State variables
    bool has_goal_;
    bool in_hold_state_;
    bool controllers_enabled_;
    double goal_x_, goal_y_, goal_yaw_;
    double robot_x_, robot_y_, robot_yaw_;
    double robot_v_;
    double hold_x_, hold_y_, hold_yaw_;
    ros::Time hold_start_time_;
    std::vector<std::string> wheel_controllers_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_stop_manager");
    GoalStopManager manager;
    manager.spin();
    return 0;
}
