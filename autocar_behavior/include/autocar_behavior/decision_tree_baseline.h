#pragma once
#include <cstdint>
#include <string>

enum class BehaviorAction : uint8_t {
    FOLLOW_PATH = 0,
    FORWARD_ALIGNMENT = 1,
    REVERSE_ALIGNMENT = 2,
    U_TURN = 3,
    THREE_POINT_TURN = 4,
    REPLAN = 5,
    RECOVERY = 6,
    GOAL_ALIGNMENT = 7,
    STOP = 8
};

inline std::string actionToString(BehaviorAction action) {
    switch (action) {
        case BehaviorAction::FOLLOW_PATH: return "FOLLOW_PATH";
        case BehaviorAction::FORWARD_ALIGNMENT: return "FORWARD_ALIGNMENT";
        case BehaviorAction::REVERSE_ALIGNMENT: return "REVERSE_ALIGNMENT";
        case BehaviorAction::U_TURN: return "U_TURN";
        case BehaviorAction::THREE_POINT_TURN: return "THREE_POINT_TURN";
        case BehaviorAction::REPLAN: return "REPLAN";
        case BehaviorAction::RECOVERY: return "RECOVERY";
        case BehaviorAction::GOAL_ALIGNMENT: return "GOAL_ALIGNMENT";
        case BehaviorAction::STOP: return "STOP";
        default: return "UNKNOWN";
    }
}

class DecisionTreeBaseline {
public:
    DecisionTreeBaseline() = default;
    ~DecisionTreeBaseline() = default;

    BehaviorAction evaluate(
        double heading_error_rad,
        double goal_dist_m,
        double front_clearance_m,
        double rear_clearance_m,
        double amcl_covariance_norm,
        int recovery_counter
    );
};
