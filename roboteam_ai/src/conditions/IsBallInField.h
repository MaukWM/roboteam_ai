//
// Created by thijs on 24-1-19.
//

#include "Condition.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"

#ifndef ROBOTEAM_AI_ISBALLINFIELD_H
#define ROBOTEAM_AI_ISBALLINFIELD_H

namespace rtt {
namespace ai {

class IsBallInField : public Condition {
    private:
        float margin = 0.0f;
    public:
        explicit IsBallInField(std::string name = "IsBallInField", bt::Blackboard::Ptr blackboard = nullptr);
        void initialize() override;
        Status update() override;
        std::string node_name() override;
};

} //ai
} //rtt

#endif //ROBOTEAM_AI_ISBALLINFIELD_H
