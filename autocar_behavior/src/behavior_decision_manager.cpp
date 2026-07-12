#include "autocar_behavior/behavior_decision_manager.h"
#include <tf2/utils.h>
#include <cmath>

BehaviorDecisionManager::BehaviorDecisionManager(ros::NodeHandle& nh)
    : nh_(nh),
      tf_listener_(tf_buffer_),
      as_(nh_, "navigate_behavior", boost::bind(&BehaviorDecisionManager::executeCB, this, _1), false),
      move_base_client_("move_base", true) {
      
    // Subscribe to localization and sensor topics
    sub_odom_ = nh_.subscribe("/diff_drive_controller/odom", 10, &BehaviorDecisionManager::odomCallback, this);
    sub_scan_ = nh_.subscribe("/scan", 10, &BehaviorDecisionManager::scanCallback, this);
    sub_amcl_pose_ = nh_.subscribe("/amcl_pose", 10, &BehaviorDecisionManager::amclPoseCallback, this);

    // Initialize RL Client
    client_rl_action_ = nh_.serviceClient<autocar_interfaces::GetRLAction>("/get_rl_action");
    nh_.param("use_rl", use_rl_, false);
    nh_.param("confidence_threshold", confidence_threshold_, 0.4);

    // Advertise the behavior state topic for logging
    pub_behavior_state_ = nh_.advertise<autocar_interfaces::BehaviorState>("/behavior_state", 10);

    // Wait for move_base action server
    ROS_INFO("[BehaviorDecisionManager] Waiting for move_base action server...");
    move_base_client_.waitForServer();
    ROS_INFO("[BehaviorDecisionManager] Connected to move_base action server.");

    // Start behavior action server
    as_.start();
    ROS_INFO("[BehaviorDecisionManager] Behavior Decision Manager Action Server started.");
}

void BehaviorDecisionManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_velocity_ = msg->twist.twist.linear.x;
    
    // Estimate steering angle from yaw rate and linear velocity
    double L = 0.70; // Wheelbase
    double w = msg->twist.twist.angular.z;
    if (std::abs(current_velocity_) > 0.05) {
        current_steering_angle_ = std::atan2(w * L, std::abs(current_velocity_));
    } else {
        current_steering_angle_ = 0.0;
    }
}

void BehaviorDecisionManager::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (msg->ranges.empty()) return;
    
    int num_samples = msg->ranges.size();
    
    // Assume 360-degree LiDAR with 720 samples
    // Index 360 is front (0 rad).
    // Index 540 is left (pi/2 rad).
    // Index 180 is right (-pi/2 rad).
    // Index 0/719 is rear (pi/-pi rad).
    
    double min_f = 30.0, min_r = 30.0, min_l = 30.0, min_rgt = 30.0;
    
    for (int i = 0; i < num_samples; ++i) {
        double r = msg->ranges[i];
        if (std::isnan(r) || std::isinf(r) || r < msg->range_min || r > msg->range_max) {
            continue;
        }
        
        // Convert index to angle relative to front
        double angle = msg->angle_min + i * msg->angle_increment;
        
        if (angle >= -0.52 && angle <= 0.52) { // [-30 deg, +30 deg] -> Front
            if (r < min_f) min_f = r;
        } else if (angle >= 1.05 && angle <= 2.09) { // [60 deg, 120 deg] -> Left
            if (r < min_l) min_l = r;
        } else if (angle >= -2.09 && angle <= -1.05) { // [-120 deg, -60 deg] -> Right
            if (r < min_rgt) min_rgt = r;
        } else if (angle <= -2.61 || angle >= 2.61) { // <-150 deg or >150 deg -> Rear
            if (r < min_r) min_r = r;
        }
    }
    
    front_clearance_ = min_f;
    left_clearance_ = min_l;
    right_clearance_ = min_rgt;
    rear_clearance_ = min_r;
}

void BehaviorDecisionManager::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // Trace of covariance matrix (x, y, and yaw)
    amcl_covariance_norm_ = msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[35];
}

bool BehaviorDecisionManager::updateRobotPose() {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
        current_pose_.header = transform.header;
        current_pose_.pose.position.x = transform.transform.translation.x;
        current_pose_.pose.position.y = transform.transform.translation.y;
        current_pose_.pose.position.z = transform.transform.translation.z;
        current_pose_.pose.orientation = transform.transform.rotation;
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("[BehaviorDecisionManager] TF Lookup failed: %s", ex.what());
        return false;
    }
}

double BehaviorDecisionManager::getHeadingError(const geometry_msgs::PoseStamped& current, const geometry_msgs::PoseStamped& goal) {
    double current_yaw = tf2::getYaw(current.pose.orientation);
    double goal_yaw = tf2::getYaw(goal.pose.orientation);
    double diff = goal_yaw - current_yaw;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

double BehaviorDecisionManager::getDistance(const geometry_msgs::PoseStamped& current, const geometry_msgs::PoseStamped& goal) {
    double dx = goal.pose.position.x - current.pose.position.x;
    double dy = goal.pose.position.y - current.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

void BehaviorDecisionManager::publishState(const geometry_msgs::PoseStamped& goal_pose, BehaviorAction action, double confidence, double latency, const std::vector<uint8_t>& alternatives, bool fallback_triggered) {
    autocar_interfaces::BehaviorState state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.header.frame_id = "map";
    state_msg.current_pose = current_pose_;
    state_msg.goal_pose = goal_pose;
    state_msg.distance_to_goal = getDistance(current_pose_, goal_pose);
    state_msg.heading_error = getHeadingError(current_pose_, goal_pose);
    state_msg.front_clearance = front_clearance_;
    state_msg.rear_clearance = rear_clearance_;
    state_msg.left_clearance = left_clearance_;
    state_msg.right_clearance = right_clearance_;
    state_msg.current_steering_angle = current_steering_angle_;
    state_msg.current_velocity = current_velocity_;
    state_msg.amcl_covariance_norm = amcl_covariance_norm_;
    state_msg.recovery_counter = recovery_counter_;
    state_msg.last_action = static_cast<uint8_t>(action);
    
    // Construct the state vector (12 features)
    state_msg.state_vector.push_back(state_msg.distance_to_goal);
    state_msg.state_vector.push_back(state_msg.heading_error);
    state_msg.state_vector.push_back(front_clearance_);
    state_msg.state_vector.push_back(rear_clearance_);
    state_msg.state_vector.push_back(left_clearance_);
    state_msg.state_vector.push_back(right_clearance_);
    state_msg.state_vector.push_back(current_steering_angle_);
    state_msg.state_vector.push_back(current_velocity_);
    state_msg.state_vector.push_back(amcl_covariance_norm_);
    state_msg.state_vector.push_back(static_cast<double>(recovery_counter_));
    state_msg.state_vector.push_back(current_pose_.pose.position.x);
    state_msg.state_vector.push_back(current_pose_.pose.position.y);
    
    state_msg.confidence = confidence;
    state_msg.decision_latency = latency;
    state_msg.alternative_actions = alternatives;
    state_msg.fallback_triggered = fallback_triggered;
    
    pub_behavior_state_.publish(state_msg);
}

void BehaviorDecisionManager::executeCB(const autocar_interfaces::NavigateBehaviorGoalConstPtr& goal) {
    ros::Rate rate(10.0); // 10Hz control loop
    
    ROS_INFO("[BehaviorDecisionManager] New goal received.");
    
    bool completed = false;
    geometry_msgs::PoseStamped active_goal = goal->target_goal;

    while (ros::ok() && as_.isActive()) {
        if (as_.isPreemptRequested()) {
            move_base_client_.cancelGoal();
            as_.setPreempted();
            ROS_INFO("[BehaviorDecisionManager] Goal preempted.");
            return;
        }

        if (!updateRobotPose()) {
            rate.sleep();
            continue;
        }

        double dist = getDistance(current_pose_, active_goal);
        double heading_err = getHeadingError(current_pose_, active_goal);

        // Evaluate action selection (RL policy server or baseline decision tree)
        BehaviorAction action;
        double confidence = 1.0;
        double latency = 0.0;
        std::vector<uint8_t> alternatives;
        bool fallback_triggered = false;

        if (use_rl_) {
            autocar_interfaces::GetRLAction srv;
            srv.request.state_vector.push_back(dist);
            srv.request.state_vector.push_back(heading_err);
            srv.request.state_vector.push_back(front_clearance_);
            srv.request.state_vector.push_back(rear_clearance_);
            srv.request.state_vector.push_back(left_clearance_);
            srv.request.state_vector.push_back(right_clearance_);
            srv.request.state_vector.push_back(current_steering_angle_);
            srv.request.state_vector.push_back(current_velocity_);
            srv.request.state_vector.push_back(amcl_covariance_norm_);
            srv.request.state_vector.push_back(static_cast<double>(recovery_counter_));
            srv.request.state_vector.push_back(current_pose_.pose.position.x);
            srv.request.state_vector.push_back(current_pose_.pose.position.y);

            ros::Time start_time = ros::Time::now();
            if (client_rl_action_.call(srv)) {
                latency = (ros::Time::now() - start_time).toSec();
                action = static_cast<BehaviorAction>(srv.response.action);
                confidence = srv.response.confidence;
                alternatives = srv.response.alternative_actions;

                // Threshold confidence check
                if (confidence < confidence_threshold_) {
                    fallback_triggered = true;
                    ROS_WARN_THROTTLE(1.0, "[BehaviorDecisionManager] RL Policy Confidence is low (%f < %f). Falling back to Baseline Tree.", confidence, confidence_threshold_);
                    action = baseline_tree_.evaluate(heading_err, dist, front_clearance_, rear_clearance_, amcl_covariance_norm_, recovery_counter_);
                }
            } else {
                latency = (ros::Time::now() - start_time).toSec();
                ROS_WARN_THROTTLE(2.0, "[BehaviorDecisionManager] RL Service call failed! Falling back to Baseline Tree.");
                action = baseline_tree_.evaluate(heading_err, dist, front_clearance_, rear_clearance_, amcl_covariance_norm_, recovery_counter_);
            }
        } else {
            action = baseline_tree_.evaluate(heading_err, dist, front_clearance_, rear_clearance_, amcl_covariance_norm_, recovery_counter_);
        }

        // Record last selected action
        last_action_ = action;

        // Publish current state to topic for experience logging
        publishState(active_goal, action, confidence, latency, alternatives, fallback_triggered);

        // Provide Feedback
        autocar_interfaces::NavigateBehaviorFeedback feedback;
        feedback.current_behavior = actionToString(action);
        feedback.distance_to_goal = dist;
        feedback.heading_error = heading_err;
        as_.publishFeedback(feedback);

        // Execute behavior action
        if (action == BehaviorAction::STOP) {
            move_base_client_.cancelGoal();
            autocar_interfaces::NavigateBehaviorResult result;
            result.success = true;
            result.message = "Successfully arrived and stopped.";
            as_.setSucceeded(result);
            ROS_INFO("[BehaviorDecisionManager] Arrived at target. Mission Complete.");
            return;
        }

        if (action == BehaviorAction::RECOVERY) {
            ROS_WARN("[BehaviorDecisionManager] Activating Recovery behavior...");
            move_base_client_.cancelGoal();
            recovery_counter_++;
            // Publish brief reversing command
            ros::Duration(1.5).sleep();
        } else if (action == BehaviorAction::REPLAN) {
            ROS_WARN("[BehaviorDecisionManager] Activating Replan behavior...");
            move_base_client_.cancelGoal();
            recovery_counter_ = 0;
            ros::Duration(1.0).sleep();
        } else {
            // Forward, Reverse, Goal alignment or standard Path following
            // Construct move_base target goal based on intermediate behaviors
            move_base_msgs::MoveBaseGoal mb_goal;
            mb_goal.target_pose = active_goal;

            if (action == BehaviorAction::FORWARD_ALIGNMENT) {
                // Shift target pose slightly forward in the robot's local heading
                double yaw = tf2::getYaw(current_pose_.pose.orientation);
                mb_goal.target_pose.pose.position.x = current_pose_.pose.position.x + 1.0 * std::cos(yaw);
                mb_goal.target_pose.pose.position.y = current_pose_.pose.position.y + 1.0 * std::sin(yaw);
            } else if (action == BehaviorAction::REVERSE_ALIGNMENT) {
                // Shift target pose slightly backward in the robot's local heading
                double yaw = tf2::getYaw(current_pose_.pose.orientation);
                mb_goal.target_pose.pose.position.x = current_pose_.pose.position.x - 1.0 * std::cos(yaw);
                mb_goal.target_pose.pose.position.y = current_pose_.pose.position.y - 1.0 * std::sin(yaw);
            }

            move_base_client_.sendGoal(mb_goal);
        }

        rate.sleep();
    }
}
