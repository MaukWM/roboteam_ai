//
// Created by baris on 6-2-19.
//

#ifndef ROBOTEAM_AI_HALTTACTIC_H
#define ROBOTEAM_AI_HALTTACTIC_H

#include <roboteam_ai/src/bt/Tactic.h>
namespace bt {

class HaltTactic : public Tactic {
    public:
        HaltTactic(std::string name, Blackboard::Ptr blackboard);
        void initialize() override;
        Node::Status update() override;
        void claimRobots();};

} // bt

#endif //ROBOTEAM_AI_HALTTACTIC_H
