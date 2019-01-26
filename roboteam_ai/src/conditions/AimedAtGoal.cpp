//
// Created by rolf on 26-1-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "AimedAtGoal.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"
namespace rtt{
namespace ai{
AimedAtGoal::AimedAtGoal(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name,blackboard){ }

bt::Node::Status AimedAtGoal::update() {
    robot=getRobotFromProperties(properties);
    ball=World::getBall();
    if (!robot || !ball) return Status::Failure;

}
std::string AimedAtGoal::node_name() {return "AimedAtGoal"; }
bool AimedAtGoal::checkAim(){
    auto field=Field::get_field();
    Vector2 goalLineStart=Vector2(field.field_length/2,-field.goal_width);
    Vector2 goalLineEnd=Vector2(field.field_length/2,field.goal_width);
    Vector2 ballAimPos=Vector2(robot->pos)+(Vector2(ball->pos)-Vector2(robot->pos)).stretchToLength(field.field_length);
    return Control::lineSegmentsIntersect(goalLineStart,goalLineEnd,robot->pos,ballAimPos);
}
}
}