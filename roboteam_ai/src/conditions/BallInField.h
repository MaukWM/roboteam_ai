//
// Created by rolf on 26-1-19.
//

#ifndef ROBOTEAM_AI_BALLINFIELD_H
#define ROBOTEAM_AI_BALLINFIELD_H
#include "Condition.h"
namespace rtt{
namespace ai{
class BallInField : public Condition{
    public:
        explicit BallInField(std::string name = "BallInField", bt::Blackboard::Ptr blackboard = nullptr);
        Status update() override;
        std::string node_name() override;
};
}
}


#endif //ROBOTEAM_AI_BALLINFIELD_H
