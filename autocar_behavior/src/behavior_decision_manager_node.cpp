#include <ros/ros.h>
#include "autocar_behavior/behavior_decision_manager.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_decision_manager");
    ros::NodeHandle nh;

    BehaviorDecisionManager manager(nh);

    ros::spin();
    return 0;
}
