#include "autocar_behavior/decision_tree_baseline.h"
#include <cmath>

BehaviorAction DecisionTreeBaseline::evaluate(
    double heading_error_rad,
    double goal_dist_m,
    double front_clearance_m,
    double rear_clearance_m,
    double amcl_covariance_norm,
    int recovery_counter
) {
    // Normalizing heading error to [0, M_PI]
    double abs_heading_error = std::abs(heading_error_rad);
    while (abs_heading_error > M_PI) {
        abs_heading_error -= 2.0 * M_PI;
    }
    abs_heading_error = std::abs(abs_heading_error);

    // 1. Goal reaching criteria
    if (goal_dist_m < 0.3) {
        if (abs_heading_error > 0.15) {
            return BehaviorAction::GOAL_ALIGNMENT;
        } else {
            return BehaviorAction::STOP;
        }
    }

    // 2. High uncertainty/failure recovery
    if (amcl_covariance_norm > 1.5) {
        return BehaviorAction::RECOVERY;
    }
    if (recovery_counter > 3) {
        return BehaviorAction::REPLAN;
    }

    // 3. Obstacle avoidance rules
    if (front_clearance_m < 0.8) {
        if (rear_clearance_m > 1.0) {
            return BehaviorAction::REVERSE_ALIGNMENT;
        } else {
            return BehaviorAction::RECOVERY;
        }
    }

    // 4. Turning maneuvers based on heading error
    if (abs_heading_error > 2.09) { // > 120 degrees
        if (front_clearance_m > 3.0 && rear_clearance_m > 3.0) {
            return BehaviorAction::U_TURN;
        } else {
            return BehaviorAction::THREE_POINT_TURN;
        }
    }

    if (abs_heading_error > 0.785) { // > 45 degrees
        return BehaviorAction::FORWARD_ALIGNMENT;
    }

    // Default: track the path
    return BehaviorAction::FOLLOW_PATH;
}
