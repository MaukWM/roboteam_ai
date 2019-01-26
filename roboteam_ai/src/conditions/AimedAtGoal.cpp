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
    if (checkAim()) return Status::Success;
    else return Status::Failure;
}
std::string AimedAtGoal::node_name() {return "AimedAtGoal"; }
bool AimedAtGoal::checkAim(){
    auto field=Field::get_field();
    Vector2 goalLineStart=Vector2(field.field_length/2,-field.goal_width/2);
    Vector2 goalLineEnd=Vector2(field.field_length/2,field.goal_width/2);
    Vector2 ballAimPosFromCentre=Vector2(robot->pos)+(Vector2(ball->pos)-Vector2(robot->pos)).stretchToLength(field.field_length);
    Vector2 ballAimPosRobotOrient=Vector2(robot->pos)+Vector2(field.field_length,0).rotate(robot->angle);
    Vector2 ballAimPos=(ballAimPosFromCentre*0.4+ballAimPosRobotOrient*0.6);
    return Control::lineSegmentsIntersect(goalLineStart,goalLineEnd,robot->pos,ballAimPos);
}
}
}