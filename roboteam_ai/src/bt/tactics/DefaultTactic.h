//
// Created by baris on 29-11-18.
//

#ifndef ROBOTEAM_AI_DEFAULTTACTIC_H
#define ROBOTEAM_AI_DEFAULTTACTIC_H

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "../Tactic.h"

namespace bt {

class DefaultTactic : public Tactic {
        using tactic = robotDealer::TacticData;
    public:
        int robotsNeeded = -1;
        int maxRobots = -1;
        std::map<std::string, robotType> robots;
        DefaultTactic(tactic me, Blackboard::Ptr blackboard);
        void initialize() override;
        Node::Status update() override;
        void claimRobots();
        struct Tactic {
          std::string name;
          int minRobots;
          std::map<std::string, robotType> roles;

        };
};
}

#endif //ROBOTEAM_AI_DEFAULTTACTIC_H
