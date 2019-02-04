//
// Created by baris on 29-11-18.
//

#include "DefaultTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = robotDealer::RobotDealer;

bt::Node::Status bt::DefaultTactic::update() {
    if (claimedRobots != robotsNeeded) {
        claimRobots();
        status = Status::Waiting;
    }
    // try to get the optional robots
    if (needsOptional) {

    }
    else {
        auto status = child->tick();

        if (status == Status::Success) {
            return Status::Success;
        }

        else {
            return Status::Running;
        }
    }
    return status;
}


bt::DefaultTactic::DefaultTactic(std::string name, bt::Blackboard::Ptr blackboard,
        std::map<std::string, robotType> robots_) {

    robots = std::move(robots_);
    globalBB = std::move(blackboard);
    this->name = std::move(name);
    robotsNeeded = static_cast<int>(robots.size());
}

void bt::DefaultTactic::initialize() {
    claimRobots();
}

void bt::DefaultTactic::claimRobots() {

    for (const auto &role : robots) {
        auto type = role.second;
        if (type == robotDealer::RobotType::optional) {
            // Keep count of the optional robots
            needsOptional++;
            continue;
        }
        robotIDs.insert(dealer::claimRobotForTactic(role.second, name, role.first));
        if (robotIDs.find(- 1) == robotIDs.end()) claimedRobots ++;
        else robotIDs.erase(- 1);
    }
}
void bt::DefaultTactic::claimOptionalRobots() {

    for (const auto &role : robots) {
        auto type = role.second;
        if (type == robotDealer::RobotType::optional) {
            if (dealer::claimRobotForOptionalTactic(name, role.first) != -1) {
                // Actually got an optional robot
                needsOptional--;
            }


        }
    }

}




