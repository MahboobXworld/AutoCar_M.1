#include <ros/ros.h>
#include <autocar_interfaces/BehaviorState.h>
#include <autocar_interfaces/Situation.h>
#include <sqlite3.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ros/package.h>

class ExperienceLogger {
public:
    ExperienceLogger(ros::NodeHandle& nh) : nh_(nh) {
        // Open/Create SQLite Database in the database directory of the package
        std::string db_path = ros::package::getPath("autocar_ai") + "/database/fleet_learning.db";
        int rc = sqlite3_open(db_path.c_str(), &db_);
        if (rc) {
            ROS_ERROR("[ExperienceLogger] Can't open database: %s", sqlite3_errmsg(db_));
        } else {
            ROS_INFO("[ExperienceLogger] Opened database successfully at %s", db_path.c_str());
            createTables();
        }

        // Subscribe to behavior state topic and situation topic
        sub_behavior_state_ = nh_.subscribe("/behavior_state", 10, &ExperienceLogger::stateCallback, this);
        sub_situation_ = nh_.subscribe("/navigation_situation", 10, &ExperienceLogger::situationCallback, this);
        
        // Generate a random mission ID for the first runs
        mission_id_ = "mission_" + std::to_string(ros::Time::now().toSec());
        step_index_ = 0;
        has_last_state_ = false;
        current_situation_ = "Unknown Situation";
    }

    ~ExperienceLogger() {
        if (db_) {
            sqlite3_close(db_);
        }
    }

private:
    void createTables() {
        std::string sql_missions = 
            "CREATE TABLE IF NOT EXISTS mission_logs ("
            "mission_id TEXT PRIMARY KEY, "
            "success INTEGER, "
            "step_count INTEGER, "
            "total_reward REAL"
            ");";
            
        std::string sql_transitions = 
            "CREATE TABLE IF NOT EXISTS transition_tuples ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
            "mission_id TEXT, "
            "step_index INTEGER, "
            "state_vector TEXT, "
            "action INTEGER, "
            "reward REAL, "
            "next_state_vector TEXT, "
            "terminal INTEGER"
            ");";

        char* zErrMsg = 0;
        sqlite3_exec(db_, sql_missions.c_str(), nullptr, 0, &zErrMsg);
        sqlite3_exec(db_, sql_transitions.c_str(), nullptr, 0, &zErrMsg);
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
    }

    void stateCallback(const autocar_interfaces::BehaviorState::ConstPtr& msg) {
        // Check if a new mission has started (goal distance changed significantly)
        if (has_last_state_ && std::abs(msg->distance_to_goal - last_state_.distance_to_goal) > 2.0) {
            // Save final summary of last mission
            saveMissionSummary(1, step_index_, total_reward_);
            
            // Start new mission
            mission_id_ = "mission_" + std::to_string(ros::Time::now().toSec());
            step_index_ = 0;
            total_reward_ = 0.0;
            has_last_state_ = false;
        }

        if (has_last_state_) {
            // 1. Calculate Reward function
            double r_progress = 2.0 * (last_state_.distance_to_goal - msg->distance_to_goal);
            
            double min_clearance = std::min({msg->front_clearance, msg->rear_clearance, msg->left_clearance, msg->right_clearance});
            double r_safety = -1.0 * std::exp(-min_clearance);
            
            double p_collision = 0.0;
            if (min_clearance < 0.20) {
                p_collision = -10.0;
            }
            
            double p_oscillation = 0.0;
            if (msg->last_action != last_state_.last_action) {
                p_oscillation = -0.5;
            }
            
            double p_timeout = -0.1; // time penalty per step

            // Context-dependent adaptive reward addition
            double r_adaptive = 0.0;
            if (current_situation_ == "Loading Dock") {
                if (msg->distance_to_goal < 1.5) {
                    r_adaptive = 3.0 * std::cos(msg->heading_error);
                }
            } else if (current_situation_ == "Wide Open Area") {
                r_adaptive = 2.0 * msg->current_velocity;
            } else if (current_situation_ == "Narrow Corridor") {
                r_adaptive = -2.0 * std::exp(-min_clearance) - 1.0 * std::abs(msg->left_clearance - msg->right_clearance);
            } else if (current_situation_ == "Dead End") {
                double dist_change = last_state_.distance_to_goal - msg->distance_to_goal;
                r_adaptive = dist_change > 0 ? 1.5 * dist_change : -0.5;
            } else if (current_situation_ == "Dynamic Obstacle Ahead") {
                r_adaptive = -0.5 * std::abs(msg->current_steering_angle) + 1.0 * min_clearance;
            }

            double step_reward = r_progress + r_safety + p_collision + p_oscillation + p_timeout + r_adaptive;
            total_reward_ += step_reward;

            // Determine if terminal
            int terminal = (msg->last_action == 8) ? 1 : 0; // Action index 8 is STOP

            // 2. Insert into database
            std::string state_str = vectorToString(last_state_.state_vector);
            std::string next_state_str = vectorToString(msg->state_vector);

            std::stringstream ss;
            ss << "INSERT INTO transition_tuples (mission_id, step_index, state_vector, action, reward, next_state_vector, terminal) VALUES ("
               << "'" << mission_id_ << "', "
               << step_index_ << ", "
               << "'" << state_str << "', "
               << static_cast<int>(last_state_.last_action) << ", "
               << step_reward << ", "
               << "'" << next_state_str << "', "
               << terminal << ");";

            char* zErrMsg = 0;
            sqlite3_exec(db_, ss.str().c_str(), nullptr, 0, &zErrMsg);
            
            step_index_++;

            if (terminal == 1) {
                saveMissionSummary(1, step_index_, total_reward_);
                has_last_state_ = false;
                return;
            }
        }

        last_state_ = *msg;
        has_last_state_ = true;
    }

    void saveMissionSummary(int success, int step_count, double total_reward) {
        std::stringstream ss;
        ss << "INSERT OR REPLACE INTO mission_logs (mission_id, success, step_count, total_reward) VALUES ("
           << "'" << mission_id_ << "', "
           << success << ", "
           << step_count << ", "
           << total_reward << ");";
           
        char* zErrMsg = 0;
        sqlite3_exec(db_, ss.str().c_str(), nullptr, 0, &zErrMsg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_behavior_state_;
    ros::Subscriber sub_situation_;
    sqlite3* db_{nullptr};

    std::string mission_id_;
    int step_index_;
    double total_reward_;
    bool has_last_state_;
    autocar_interfaces::BehaviorState last_state_;
    std::string current_situation_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "experience_logger");
    ros::NodeHandle nh;
    ExperienceLogger logger(nh);
    ros::spin();
    return 0;
}
