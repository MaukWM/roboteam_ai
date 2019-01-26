//
// Created by rolf on 26-1-19.
//

#include "BallInField.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"
namespace rtt{
namespace ai{
BallInField::BallInField(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name,blackboard){ }

bt::Node::Status BallInField::update() {
    ball=World::getBall();
    if ( !ball) return Status::Failure;
    if (Field::pointIsInField(ball->pos,0.0)){ return Status::Success;}
    return Status::Failure;
}
std::string BallInField::node_name() {return "BallInField"; }
}
}