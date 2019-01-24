//
// Created by thijs on 24-1-19.
//

#include "IsBallInField.h"

namespace rtt {
namespace ai {

IsBallInField::IsBallInField(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsBallInField::initialize() {

    if (properties->hasBool("margin"))
        margin = static_cast<float>(properties->getBool("margin"));
    else
        margin = 0.0f;
}

bt::Node::Status IsBallInField::update() {
    Vector2 ballPos = World::getBall().get()->pos;
    return Field::pointIsInField(ballPos, margin) ?
    Status::Success : Status::Failure;
}

std::string IsBallInField::node_name() { return "IsBallInField"; }

}
}

