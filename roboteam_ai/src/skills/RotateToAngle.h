//
// Created by rolf on 21/11/18.
//

#ifndef ROBOTEAM_AI_ROTATETOANGLE_H
#define ROBOTEAM_AI_ROTATETOANGLE_H

#include "Skill.h"

namespace rtt {
namespace ai {

class RotateToAngle : public Skill {
    private:
        double targetAngle, deltaAngle;
        bool useAngle;
        enum Progression {
          ROTATING, DONE, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();
        std::chrono::system_clock::time_point timer;
        std::chrono::duration<double> timeDiff;

    public:
        explicit RotateToAngle(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};
}//ai
}//rtt


#endif //ROBOTEAM_AI_ROTATETOANGLE_H
