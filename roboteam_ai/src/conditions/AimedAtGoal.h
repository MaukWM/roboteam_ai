//
// Created by rolf on 26-1-19.
//

#ifndef ROBOTEAM_AI_AIMEDATGOAL_H
#define ROBOTEAM_AI_AIMEDATGOAL_H

#include "Condition.h"
namespace rtt{
namespace ai{
class AimedAtGoal : public Condition {
    private:
        bool checkAim();
    public:
        explicit AimedAtGoal(std::string name = "AimedAtGoal", bt::Blackboard::Ptr blackboard = nullptr);
        Status update() override;
        std::string node_name() override;
};
}
}


#endif //ROBOTEAM_AI_AIMEDATGOAL_H
