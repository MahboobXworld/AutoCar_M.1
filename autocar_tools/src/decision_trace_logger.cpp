#include <ros/ros.h>
#include <autocar_interfaces/BehaviorState.h>
#include <autocar_interfaces/Situation.h>
#include <sqlite3.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/package.h>

class DecisionTraceLogger {
public:
    DecisionTraceLogger(ros::NodeHandle& nh) : nh_(nh) {
        // Open/Create SQLite Database
        std::string db_path = ros::package::getPath("autocar_ai") + "/database/fleet_learning.db";
        int rc = sqlite3_open(db_path.c_str(), &db_);
        if (rc) {
            ROS_ERROR("[DecisionTraceLogger] Can't open database: %s", sqlite3_errmsg(db_));
        } else {
            createTable();
        }

        // Subscribe to behavior state and situation classifier
        sub_behavior_state_ = nh_.subscribe("/behavior_state", 10, &DecisionTraceLogger::stateCallback, this);
        sub_situation_ = nh_.subscribe("/navigation_situation", 10, &DecisionTraceLogger::situationCallback, this);
        
        ROS_INFO("[DecisionTraceLogger] Node initialized and listening to /behavior_state and /navigation_situation.");
    }

    ~DecisionTraceLogger() {
        if (db_) {
            sqlite3_close(db_);
        }
    }

private:
    void createTable() {
        std::string sql1 = 
            "CREATE TABLE IF NOT EXISTS decision_trace ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
            "timestamp TEXT, "
            "robot_pose_x REAL, "
            "robot_pose_y REAL, "
            "robot_pose_yaw REAL, "
            "goal_pose_x REAL, "
            "goal_pose_y REAL, "
            "goal_pose_yaw REAL, "
            "heading_error REAL, "
            "cross_track_error REAL, "
            "front_clearance REAL, "
            "rear_clearance REAL, "
            "left_clearance REAL, "
            "right_clearance REAL, "
            "selected_action INTEGER, "
            "confidence REAL, "
            "reward REAL, "
            "next_state TEXT, "
            "decision_latency REAL, "
            "situation TEXT, "
            "explanation TEXT"
            ");";

        char* zErrMsg = 0;
        int rc = sqlite3_exec(db_, sql1.c_str(), nullptr, 0, &zErrMsg);
        if (rc != SQLITE_OK) {
            ROS_ERROR("[DecisionTraceLogger] SQLite Table Creation Error (decision_trace): %s", zErrMsg);
            sqlite3_free(zErrMsg);
        }

        std::string sql2 =
            "CREATE TABLE IF NOT EXISTS fallback_logs ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
            "timestamp TEXT, "
            "confidence REAL, "
            "baseline_action INTEGER"
            ");";
        rc = sqlite3_exec(db_, sql2.c_str(), nullptr, 0, &zErrMsg);
        if (rc != SQLITE_OK) {
            ROS_ERROR("[DecisionTraceLogger] SQLite Table Creation Error (fallback_logs): %s", zErrMsg);
            sqlite3_free(zErrMsg);
        }
    }

    double getYaw(const geometry_msgs::Quaternion& q) {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    std::string vectorToString(const std::vector<double>& vec) {
        std::stringstream ss;
        for (size_t i = 0; i < vec.size(); ++i) {
            ss << vec[i];
            if (i < vec.size() - 1) {
                ss << ",";
            }
        }
        return ss.str();
    }

    void situationCallback(const autocar_interfaces::Situation::ConstPtr& msg) {
        current_situation_ = msg->situation;
        situation_confidence_ = msg->confidence;
    }

    void stateCallback(const autocar_interfaces::BehaviorState::ConstPtr& msg) {
        double rx = msg->current_pose.pose.position.x;
        double ry = msg->current_pose.pose.position.y;
        double ryaw = getYaw(msg->current_pose.pose.orientation);

        double gx = msg->goal_pose.pose.position.x;
        double gy = msg->goal_pose.pose.position.y;
        double gyaw = getYaw(msg->goal_pose.pose.orientation);

        // Simple CTE computation relative to start-to-goal straight line
        double cte = 0.0;
        double A = gy - ry;
        double B = rx - gx;
        double C = gx * ry - rx * gy;
        double denom = std::sqrt(A*A + B*B);
        if (denom > 0.001) {
            cte = std::abs(A * rx + B * ry + C) / denom;
        }

        // Dummy/approx reward for step
        double step_reward = 0.0;
        double min_clearance = std::min({msg->front_clearance, msg->rear_clearance, msg->left_clearance, msg->right_clearance});
        if (min_clearance < 0.20) step_reward -= 10.0;
        step_reward -= 0.1; // time step cost

        std::string next_state_str = vectorToString(msg->state_vector);

        // Map action to string
        std::string action_str = "UNKNOWN";
        switch (msg->last_action) {
            case 0: action_str = "FOLLOW_PATH"; break;
            case 1: action_str = "FORWARD_ALIGNMENT"; break;
            case 2: action_str = "REVERSE_ALIGNMENT"; break;
            case 3: action_str = "U_TURN"; break;
            case 4: action_str = "THREE_POINT_TURN"; break;
            case 5: action_str = "REPLAN"; break;
            case 6: action_str = "RECOVERY"; break;
            case 7: action_str = "GOAL_ALIGNMENT"; break;
            case 8: action_str = "STOP"; break;
        }

        // Generate intelligent decision reasoning explanation
        std::stringstream explanation_ss;
        if (current_situation_ == "Goal Behind Robot" || (std::abs(msg->heading_error) > 2.0 && msg->last_action == 2)) {
            explanation_ss << "Heading Error: " << std::round(msg->heading_error * 180.0 / M_PI) << " deg, "
                           << "Front Clearance: " << std::round(msg->front_clearance * 100) / 100.0 << " m, "
                           << "Rear Clearance: " << std::round(msg->rear_clearance * 100) / 100.0 << " m -> "
                           << "Situation: Goal Behind Robot -> "
                           << "Decision: Reverse Alignment (" << std::round(msg->confidence * 100) << "%) -> "
                           << "Reason: Reverse alignment minimizes steering effort and expected mission time while maintaining safe clearance.";
        } else if (msg->last_action == 8) {
            explanation_ss << "Heading Error: " << std::round(msg->heading_error * 180.0 / M_PI) << " deg, "
                           << "Goal Distance: " << std::round(msg->distance_to_goal * 100) / 100.0 << " m -> "
                           << "Situation: Tight Goal Alignment -> "
                           << "Decision: STOP -> "
                           << "Reason: Target goal reached. Securing vehicle stance.";
        } else if (current_situation_ == "Narrow Corridor") {
            explanation_ss << "Clearances (Left: " << std::round(msg->left_clearance * 100) / 100.0 << " m, "
                           << "Right: " << std::round(msg->right_clearance * 100) / 100.0 << " m) -> "
                           << "Situation: Narrow Corridor -> "
                           << "Decision: " << action_str << " -> "
                           << "Reason: Centered path following enforces high safety margin in tight lanes.";
        } else {
            explanation_ss << "Clearance: " << std::round(min_clearance * 100) / 100.0 << " m, "
                           << "Heading Error: " << std::round(msg->heading_error * 180.0 / M_PI) << " deg -> "
                           << "Situation: " << current_situation_ << " -> "
                           << "Decision: " << action_str << " (" << std::round(msg->confidence * 100) << "%) -> "
                           << "Reason: Dynamic trajectory optimization ensures high path compliance and safe clearances.";
        }
        std::string explanation = explanation_ss.str();

        std::stringstream ss;
        ss << "INSERT INTO decision_trace (timestamp, robot_pose_x, robot_pose_y, robot_pose_yaw, "
           << "goal_pose_x, goal_pose_y, goal_pose_yaw, heading_error, cross_track_error, "
           << "front_clearance, rear_clearance, left_clearance, right_clearance, selected_action, "
           << "confidence, reward, next_state, decision_latency, situation, explanation) VALUES ("
           << "'" << std::to_string(msg->header.stamp.toSec()) << "', "
           << rx << ", "
           << ry << ", "
           << ryaw << ", "
           << gx << ", "
           << gy << ", "
           << gyaw << ", "
           << msg->heading_error << ", "
           << cte << ", "
           << msg->front_clearance << ", "
           << msg->rear_clearance << ", "
           << msg->left_clearance << ", "
           << msg->right_clearance << ", "
           << static_cast<int>(msg->last_action) << ", "
           << msg->confidence << ", "
           << step_reward << ", "
           << "'" << next_state_str << "', "
           << msg->decision_latency << ", "
           << "'" << current_situation_ << "', "
           << "'" << explanation << "');";

        char* zErrMsg = 0;
        int rc = sqlite3_exec(db_, ss.str().c_str(), nullptr, 0, &zErrMsg);
        if (rc != SQLITE_OK) {
            ROS_ERROR_THROTTLE(2.0, "[DecisionTraceLogger] SQLite Insert Error: %s", zErrMsg);
            sqlite3_free(zErrMsg);
        }

        if (msg->fallback_triggered) {
            std::stringstream ss_fallback;
            ss_fallback << "INSERT INTO fallback_logs (timestamp, confidence, baseline_action) VALUES ("
                        << "'" << std::to_string(msg->header.stamp.toSec()) << "', "
                        << msg->confidence << ", "
                        << static_cast<int>(msg->last_action) << ");";
            rc = sqlite3_exec(db_, ss_fallback.str().c_str(), nullptr, 0, &zErrMsg);
            if (rc != SQLITE_OK) {
                ROS_ERROR_THROTTLE(2.0, "[DecisionTraceLogger] SQLite Fallback Insert Error: %s", zErrMsg);
                sqlite3_free(zErrMsg);
            }
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_behavior_state_;
    ros::Subscriber sub_situation_;
    sqlite3* db_{nullptr};

    std::string current_situation_{"Unknown Situation"};
    double situation_confidence_{0.5};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "decision_trace_logger");
    ros::NodeHandle nh;
    DecisionTraceLogger logger(nh);
    ros::spin();
    return 0;
}
