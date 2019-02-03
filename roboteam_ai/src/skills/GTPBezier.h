//
// Created by simen on 03/02/19.
//

#ifndef ROBOTEAM_AI_GTPBEZIER_H
#define ROBOTEAM_AI_GTPBEZIER_H

#include "Skill.h"
#include "../control/pathFinder/CurveCreator.h"

namespace rtt {
namespace ai {

class GTPBezier : public Skill {
    private:

        bool goToBall;
        bool goBehindBall;
        double distanceBehindBall;
        double speed;
        enum Progression {
          ON_THE_WAY, DONE, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();

        Vector2 deltaPos;
        Vector2 targetPos;

        CurveCreator curveCreator;
        control::Controller controller;
        std::vector<Vector2> curvePos, curveVel;
        float totalTime;
        std::chrono::system_clock::time_point startTime;
        std::chrono::system_clock::time_point now;
        std::chrono::duration<double> timeDif;


        void sendMoveCommand(float xVelocity, float yVelocity);
        void sendMoveCommand2();
        bool commandSend;

    public:
        explicit GTPBezier(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
};
} // ai
} // rtt
#endif //ROBOTEAM_AI_GTPBEZIER_H
