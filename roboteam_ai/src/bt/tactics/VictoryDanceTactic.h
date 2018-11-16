//
// Created by thijs on 15-11-18.
//
#include "../Tactic.h"
#include "../../../src/utilities/RobotDealer.h"

#ifndef ROBOTEAM_AI_VICTORYDANCETACTIC_H
#define ROBOTEAM_AI_VICTORYDANCETACTIC_H


namespace bt {

class VictoryDanceTactic : public Tactic {

    public:
        VictoryDanceTactic(std::string name, Blackboard::Ptr blackboard);

        std::string name;

        void setName(std::string newName);

        void Initialize() override;
        Node::Status Update() override;

        std::string node_name() override;

        bool claimedRobots = false;

        std::set<int> robotIDs = {};

//        Node::Ptr child = nullptr;



};

#endif //ROBOTEAM_AI_VICTORYDANCETACTIC_H
} // bt