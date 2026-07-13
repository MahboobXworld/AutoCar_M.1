#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <autocar_interfaces/NavigateBehaviorAction.h>
#include <autocar_interfaces/BehaviorState.h>
#include "autocar_behavior/decision_tree_baseline.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <autocar_interfaces/GetRLAction.h>

class BehaviorDecisionManager {
public:
    BehaviorDecisionManager(ros::NodeHandle& nh);
    ~BehaviorDecisionManager() = default;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void executeCB(const autocar_interfaces::NavigateBehaviorGoalConstPtr& goal);
    
    bool updateRobotPose();
    double getHeadingError(const geometry_msgs::PoseStamped& current, const geometry_msgs::PoseStamped& goal);
    double getDistance(const geometry_msgs::PoseStamped& current, const geometry_msgs::PoseStamped& goal);
    void publishState(const geometry_msgs::PoseStamped& goal_pose, BehaviorAction action, double confidence, double latency, const std::vector<uint8_t>& alternatives, bool fallback_triggered);

    ros::NodeHandle nh_;
    
    // Publishers & Subscribers
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_amcl_pose_;
    ros::Publisher pub_behavior_state_;
    ros::ServiceClient client_rl_action_;

    // Action Servers & Clients
    actionlib::SimpleActionServer<autocar_interfaces::NavigateBehaviorAction> as_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

    // TF listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Baseline Tree
    DecisionTreeBaseline baseline_tree_;

    // State Variables
    geometry_msgs::PoseStamped current_pose_;
    double current_velocity_{0.0};
    double current_steering_angle_{0.0};
    
    double front_clearance_{30.0};
    double rear_clearance_{30.0};
    double left_clearance_{30.0};
    double right_clearance_{30.0};
    
    double amcl_covariance_norm_{0.0};
    int recovery_counter_{0};
    BehaviorAction last_action_{BehaviorAction::STOP};
    BehaviorAction last_sent_action_{BehaviorAction::STOP};
    geometry_msgs::PoseStamped last_sent_goal_;
    bool use_rl_{false};
    double confidence_threshold_{0.4};
};
