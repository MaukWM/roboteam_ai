//
// Created by baris on 24/10/18.
//

#ifndef ROBOTEAM_AI_Rotate180_H
#define ROBOTEAM_AI_Rotate180_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Rotate180 : public Skill {
    private:
        float desiredAngle;
        std::chrono::system_clock::time_point timer, now;

        void sendMoveCommand();

    public:
        explicit Rotate180(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
};
} // ai
} // rtt

#endif //ROBOTEAM_AI_Rotate180_H
