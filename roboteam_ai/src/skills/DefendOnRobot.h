//
// Created by robzelluf on 12/7/18.
//

#ifndef ROBOTEAM_AI_DEFENDONROBOT_H
#define ROBOTEAM_AI_DEFENDONROBOT_H

#include "Skill.h"
#include <boost/optional.hpp>
#include <roboteam_ai/src/conditions/HasBall.hpp>
#include <roboteam_ai/src/utilities/Coach.h>
#include <roboteam_ai/src/control/ControlGoToPos.h>

namespace rtt {
namespace ai {

class DefendOnRobot : public Skill {
    private:
        using status = bt::Node::Status;
        using goType = control::ControlGoToPos::GoToType;

        control::ControlGoToPos goToPos;

        int amountOfCycles{};
    protected:
        enum Progression {
          IDLE, DONE, FAIL
        };
        Progression currentProgress;
    public:
        explicit DefendOnRobot(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        void initialize() override;
        void terminate(Status s) override;
        Status update() override;
        Vector2 calculateLocation();
        std::shared_ptr<roboteam_msgs::WorldRobot> robot1;
        std::shared_ptr<roboteam_msgs::WorldRobot> robot2;
};
} // ai
} // rtt


#endif //ROBOTEAM_AI_DEFENDONROBOT_H
