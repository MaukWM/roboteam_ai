//
// Created by robzelluf on 1/24/19.
//

#ifndef ROBOTEAM_AI_BP_TWOROBOTS_H
#define ROBOTEAM_AI_BP_TWOROBOTS_H

#include "Condition.h"
#include "../utilities/Coach.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai {

class BP_TwoRobots : public Condition {
private:
    Vector2 BP_location;
    int robotClosestToBallID;
    int robotClosestToBpPoint;
public:
    explicit BP_TwoRobots(std::string name = "BP_TwoRobots", bt::Blackboard::Ptr blackboard = nullptr);
    void initialize() override;
    Status update() override;
    std::string node_name() override;
};

}
}


#endif //ROBOTEAM_AI_BP_TWOROBOTS_H
