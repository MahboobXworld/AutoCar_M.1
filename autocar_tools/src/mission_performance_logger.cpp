#include <ros/ros.h>
#include <autocar_interfaces/BehaviorState.h>
#include <sqlite3.h>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <ros/package.h>

class MissionPerformanceLogger {
public:
    MissionPerformanceLogger(ros::NodeHandle& nh) : nh_(nh) {
        // Read ROS parameters
        nh_.param<std::string>("policy_version", policy_version_, "baseline_v1");
        nh_.param<std::string>("robot_name", robot_name_, "autocar_01");
        nh_.param<std::string>("warehouse_map", warehouse_map_, "warehouse_world");

        // Database Path
        std::string db_path = ros::package::getPath("autocar_ai") + "/database/fleet_learning.db";
        int rc = sqlite3_open(db_path.c_str(), &db_);
        if (rc) {
            ROS_ERROR("[MissionPerformanceLogger] Can't open database: %s", sqlite3_errmsg(db_));
        } else {
            createTable();
        }

        // Subscriber to behavior state
        sub_behavior_ = nh_.subscribe("/behavior_state", 10, &MissionPerformanceLogger::behaviorCallback, this);
        
        in_mission_ = false;
        ROS_INFO("[MissionPerformanceLogger] Node initialized and ready.");
    }

    ~MissionPerformanceLogger() {
        if (db_) {
            sqlite3_close(db_);
        }
    }

private:
    void createTable() {
        std::string sql = 
            "CREATE TABLE IF NOT EXISTS mission_performance_reports ("
            "mission_id TEXT PRIMARY KEY, "
            "policy_version TEXT, "
            "robot_name TEXT, "
            "warehouse_map TEXT, "
            "start_time TEXT, "
            "end_time TEXT, "
            "duration REAL, "
            "success INTEGER, "
            "failure_reason TEXT, "
            "distance_travelled REAL, "
            "avg_speed REAL, "
            "max_speed REAL, "
            "avg_steering REAL, "
            "max_steering REAL, "
            "avg_heading_err REAL, "
            "max_heading_err REAL, "
            "avg_cross_track_err REAL, "
            "max_cross_track_err REAL, "
            "goal_position_err REAL, "
            "goal_orientation_err REAL, "
            "num_forward INTEGER, "
            "num_reverse INTEGER, "
            "num_uturn INTEGER, "
            "num_threepoint INTEGER, "
            "num_replan INTEGER, "
            "num_recovery INTEGER, "
            "collision_count INTEGER, "
            "near_collision_count INTEGER, "
            "estop_count INTEGER, "
            "avg_clearance REAL, "
            "min_clearance REAL, "
            "avg_covariance REAL, "
            "max_covariance REAL, "
            "energy_estimate REAL, "
            "total_reward REAL, "
            "avg_reward REAL, "
            "discounted_return REAL"
            ");";

        char* zErrMsg = 0;
        sqlite3_exec(db_, sql.c_str(), nullptr, 0, &zErrMsg);
    }

    double getYaw(const geometry_msgs::Quaternion& q) {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double computeCrossTrackError(double x, double y) {
        if (start_x_ == goal_x_ && start_y_ == goal_y_) return 0.0;
        // Line representation: A*x + B*y + C = 0
        double A = goal_y_ - start_y_;
        double B = start_x_ - goal_x_;
        double C = goal_x_ * start_y_ - start_x_ * goal_y_;
        return std::abs(A * x + B * y + C) / std::sqrt(A*A + B*B);
    }

    void resetMission(const autocar_interfaces::BehaviorState::ConstPtr& msg) {
        mission_id_ = "mission_" + std::to_string(ros::Time::now().toSec());
        start_time_ = ros::Time::now();
        last_step_time_ = start_time_;
        
        start_x_ = msg->current_pose.pose.position.x;
        start_y_ = msg->current_pose.pose.position.y;
        goal_x_ = msg->goal_pose.pose.position.x;
        goal_y_ = msg->goal_pose.pose.position.y;

        distance_travelled_ = 0.0;
        last_x_ = start_x_;
        last_y_ = start_y_;

        speeds_.clear();
        steering_angles_.clear();
        heading_errors_.clear();
        cross_track_errors_.clear();
        clearances_.clear();
        covariances_.clear();
        rewards_.clear();

        num_forward_ = 0;
        num_reverse_ = 0;
        num_uturn_ = 0;
        num_threepoint_ = 0;
        num_replan_ = 0;
        num_recovery_ = 0;
        collision_count_ = 0;
        near_collision_count_ = 0;
        estop_count_ = 0;
        energy_estimate_ = 0.0;
        total_reward_ = 0.0;
        discounted_return_ = 0.0;

        in_mission_ = true;
        ROS_INFO("[MissionPerformanceLogger] Mission started: %s", mission_id_.c_str());
    }

    void behaviorCallback(const autocar_interfaces::BehaviorState::ConstPtr& msg) {
        // Condition to detect start of a mission
        // 8 = STOP, 0 = FOLLOW_PATH
        if (!in_mission_ && msg->last_action != 8 && msg->distance_to_goal > 0.5) {
            resetMission(msg);
        }

        if (!in_mission_) return;

        // Calculate step duration
        ros::Time now = ros::Time::now();
        double dt = (now - last_step_time_).toSec();
        if (dt <= 0.0) dt = 0.1;
        last_step_time_ = now;

        // Core variables
        double cx = msg->current_pose.pose.position.x;
        double cy = msg->current_pose.pose.position.y;
        double speed = std::abs(msg->current_velocity);
        double steering = std::abs(msg->current_steering_angle);
        double min_clearance = std::min({msg->front_clearance, msg->rear_clearance, msg->left_clearance, msg->right_clearance});

        // 1. Distance Travelled
        double dx = cx - last_x_;
        double dy = cy - last_y_;
        distance_travelled_ += std::sqrt(dx*dx + dy*dy);
        last_x_ = cx;
        last_y_ = cy;

        // 2. Metrics Accumulation
        speeds_.push_back(speed);
        steering_angles_.push_back(steering);
        heading_errors_.push_back(std::abs(msg->heading_error));
        cross_track_errors_.push_back(computeCrossTrackError(cx, cy));
        clearances_.push_back(min_clearance);
        covariances_.push_back(msg->amcl_covariance_norm);

        // 3. Collision counts
        if (min_clearance < 0.18) {
            collision_count_++;
        } else if (min_clearance < 0.35) {
            near_collision_count_++;
        }

        // 4. Energy Estimation (Model-based: speed/steering drag)
        energy_estimate_ += (0.5 * speed + 0.1 * steering + 0.05) * dt;

        // 5. Behavior Counters
        if (msg->last_action == 1) num_forward_++;
        else if (msg->last_action == 2) num_reverse_++;
        else if (msg->last_action == 3) num_uturn_++;
        else if (msg->last_action == 4) num_threepoint_++;
        else if (msg->last_action == 5) num_replan_++;
        else if (msg->last_action == 6) num_recovery_++;
        else if (msg->last_action == 8) estop_count_++;

        // 6. Step Reward calculation
        double r_progress = 2.0 * (prev_distance_ - msg->distance_to_goal);
        double r_safety = -1.0 * std::exp(-min_clearance);
        double r_alignment = (msg->distance_to_goal < 1.0) ? 1.5 * std::cos(msg->heading_error) : 0.0;
        double p_collision = (min_clearance < 0.20) ? -10.0 : 0.0;
        double p_timeout = -0.1;
        double step_reward = r_progress + r_safety + r_alignment + p_collision + p_timeout;
        
        rewards_.push_back(step_reward);
        total_reward_ += step_reward;
        discounted_return_ += step_reward * std::pow(0.99, rewards_.size() - 1);

        prev_distance_ = msg->distance_to_goal;

        // 7. Check Mission Terminal Condition
        if (msg->last_action == 8 || msg->distance_to_goal < 0.3) {
            saveReport(msg, 1, "Goal Reached");
        } else if (msg->recovery_counter > 5) {
            saveReport(msg, 0, "Recovery Limit Exceeded");
        } else if (speeds_.size() > 1500) { // Timeout after 150 seconds
            saveReport(msg, 0, "Mission Timeout");
        }
    }

    void saveReport(const autocar_interfaces::BehaviorState::ConstPtr& msg, int success, const std::string& reason) {
        in_mission_ = false;
        double duration = (ros::Time::now() - start_time_).toSec();

        // Calculate Averages and Maximums
        double avg_speed = speeds_.empty() ? 0.0 : std::accumulate(speeds_.begin(), speeds_.end(), 0.0) / speeds_.size();
        double max_speed = speeds_.empty() ? 0.0 : *std::max_element(speeds_.begin(), speeds_.end());
        double avg_steering = steering_angles_.empty() ? 0.0 : std::accumulate(steering_angles_.begin(), steering_angles_.end(), 0.0) / steering_angles_.size();
        double max_steering = steering_angles_.empty() ? 0.0 : *std::max_element(steering_angles_.begin(), steering_angles_.end());
        double avg_heading = heading_errors_.empty() ? 0.0 : std::accumulate(heading_errors_.begin(), heading_errors_.end(), 0.0) / heading_errors_.size();
        double max_heading = heading_errors_.empty() ? 0.0 : *std::max_element(heading_errors_.begin(), heading_errors_.end());
        double avg_cte = cross_track_errors_.empty() ? 0.0 : std::accumulate(cross_track_errors_.begin(), cross_track_errors_.end(), 0.0) / cross_track_errors_.size();
        double max_cte = cross_track_errors_.empty() ? 0.0 : *std::max_element(cross_track_errors_.begin(), cross_track_errors_.end());
        double avg_clearance = clearances_.empty() ? 0.0 : std::accumulate(clearances_.begin(), clearances_.end(), 0.0) / clearances_.size();
        double min_clearance = clearances_.empty() ? 0.0 : *std::min_element(clearances_.begin(), clearances_.end());
        double avg_cov = covariances_.empty() ? 0.0 : std::accumulate(covariances_.begin(), covariances_.end(), 0.0) / covariances_.size();
        double max_cov = covariances_.empty() ? 0.0 : *std::max_element(covariances_.begin(), covariances_.end());
        double avg_reward = rewards_.empty() ? 0.0 : total_reward_ / rewards_.size();

        // Goal errors
        double goal_pos_err = msg->distance_to_goal;
        double current_yaw = getYaw(msg->current_pose.pose.orientation);
        double goal_yaw = getYaw(msg->goal_pose.pose.orientation);
        double goal_ori_err = std::abs(goal_yaw - current_yaw);

        // SQLite insert statement
        std::stringstream ss;
        ss << "INSERT INTO mission_performance_reports (mission_id, policy_version, robot_name, warehouse_map, "
           << "start_time, end_time, duration, success, failure_reason, distance_travelled, avg_speed, max_speed, "
           << "avg_steering, max_steering, avg_heading_err, max_heading_err, avg_cross_track_err, max_cross_track_err, "
           << "goal_position_err, goal_orientation_err, num_forward, num_reverse, num_uturn, num_threepoint, "
           << "num_replan, num_recovery, collision_count, near_collision_count, estop_count, avg_clearance, min_clearance, "
           << "avg_covariance, max_covariance, energy_estimate, total_reward, avg_reward, discounted_return) VALUES ("
           << "'" << mission_id_ << "', "
           << "'" << policy_version_ << "', "
           << "'" << robot_name_ << "', "
           << "'" << warehouse_map_ << "', "
           << "'" << std::to_string(start_time_.toSec()) << "', "
           << "'" << std::to_string(ros::Time::now().toSec()) << "', "
           << duration << ", "
           << success << ", "
           << "'" << reason << "', "
           << distance_travelled_ << ", "
           << avg_speed << ", "
           << max_speed << ", "
           << avg_steering << ", "
           << max_steering << ", "
           << avg_heading << ", "
           << max_heading << ", "
           << avg_cte << ", "
           << max_cte << ", "
           << goal_pos_err << ", "
           << goal_ori_err << ", "
           << num_forward_ << ", "
           << num_reverse_ << ", "
           << num_uturn_ << ", "
           << num_threepoint_ << ", "
           << num_replan_ << ", "
           << num_recovery_ << ", "
           << collision_count_ << ", "
           << near_collision_count_ << ", "
           << estop_count_ << ", "
           << avg_clearance << ", "
           << min_clearance << ", "
           << avg_cov << ", "
           << max_cov << ", "
           << energy_estimate_ << ", "
           << total_reward_ << ", "
           << avg_reward << ", "
           << discounted_return_ << ");";

        char* zErrMsg = 0;
        int rc = sqlite3_exec(db_, ss.str().c_str(), nullptr, 0, &zErrMsg);
        if (rc != SQLITE_OK) {
            ROS_ERROR("[MissionPerformanceLogger] SQLite Insert Error: %s", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("[MissionPerformanceLogger] Performance Report successfully saved to SQLite database.");
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_behavior_;
    sqlite3* db_{nullptr};

    // Mission State Variables
    bool in_mission_;
    std::string mission_id_;
    ros::Time start_time_;
    ros::Time last_step_time_;
    double start_x_, start_y_;
    double goal_x_, goal_y_;
    double prev_distance_{0.0};

    // Parameters
    std::string policy_version_;
    std::string robot_name_;
    std::string warehouse_map_;

    // Performance accumulations
    double distance_travelled_;
    double last_x_, last_y_;

    std::vector<double> speeds_;
    std::vector<double> steering_angles_;
    std::vector<double> heading_errors_;
    std::vector<double> cross_track_errors_;
    std::vector<double> clearances_;
    std::vector<double> covariances_;
    std::vector<double> rewards_;

    int num_forward_;
    int num_reverse_;
    int num_uturn_;
    int num_threepoint_;
    int num_replan_;
    int num_recovery_;
    int collision_count_;
    int near_collision_count_;
    int estop_count_;
    double energy_estimate_;
    double total_reward_;
    double discounted_return_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_performance_logger");
    ros::NodeHandle nh;
    MissionPerformanceLogger logger(nh);
    ros::spin();
    return 0;
}
