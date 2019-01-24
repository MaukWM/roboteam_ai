//
// Created by robzelluf on 1/24/19.
//

#include "BP_TwoRobots.h"

namespace rtt {
namespace ai {

BP_TwoRobots::BP_TwoRobots(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void BP_TwoRobots::initialize() {
    robotClosestToBallID = coach::Coach::getRobotClosestToBall(true)->id;
    BP_location = coach::Coach::getBallPlacementPos();
    robotClosestToBpPoint = coach::Coach::getRobotClosestToPoint(true, BP_location)->id;
};

bt::Node::Status BP_TwoRobots::update() {
    if (!robotClosestToBallID || !robotClosestToBpPoint) return Status::Failure;

    if (robotClosestToBallID == robotClosestToBpPoint){
        return Status::Failure;
    } else return Status::Success;
}

std::string BP_TwoRobots::node_name() {return "BP_TwoRobots";}

}
}
