//
// Created by baris on 15-1-19.
//

#ifndef ROBOTEAM_AI_BASICGOTOPOS_H
#define ROBOTEAM_AI_BASICGOTOPOS_H

#include <roboteam_ai/src/control/ControlGoToPos.h>
#include "Skill.h"

namespace rtt {
namespace ai {


class BasicGoToPos : public Skill {
    private:
        Vector2 targetPos;
        control::ControlGoToPos goToPos;
        double errorMargin = 0.3;
        bool goToBall = false;
        bool goBehindBall = false;
        double distanceBehindBall;
    public:
        explicit BasicGoToPos(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;



};
}
}

#endif //ROBOTEAM_AI_BASICGOTOPOS_H
