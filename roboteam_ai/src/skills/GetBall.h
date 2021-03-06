//
// Created by rolf on 04/12/18.
//

#ifndef ROBOTEAM_AI_GETBALL_H
#define ROBOTEAM_AI_GETBALL_H

#include "Skill.h"

namespace rtt {
namespace ai {

class GetBall : public Skill {
    private:
        enum Progression {
          TURNING, APPROACHING, OVERSHOOTING,DRIBBLING, SUCCESS, FAIL
        };

        Progression currentProgress;
        void checkProgression();

        bool robotHasBall(double frontRange);
        void sendTurnCommand();
        void sendApproachCommand();
        void sendDribblingCommand();
        void sendOvershootCommand();

        int count;
        Vector2 deltaPos, approachPos;
        bool lockAngle;
        double lockedAngle;
        int currentTick, maxTicks;
    public:
        explicit GetBall(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;

};
}
}

#endif //ROBOTEAM_AI_GETBALL_H
