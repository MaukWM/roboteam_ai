//
// Created by baris on 15-1-19.
//

#include "BasicGoToPos.h"

namespace rtt {
namespace ai {

BasicGoToPos::BasicGoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {

}

void BasicGoToPos::onInitialize() {
    robot = getRobotFromProperties(properties);

    if (properties->hasVector2("target"))
        targetPos = properties->getVector2("target");

    goToBall = properties->getBool("goToBall");
    goBehindBall = properties->getBool("goBehindBall");

    if (properties->hasDouble("distanceBehindBall"))
        distanceBehindBall = properties->getDouble("distanceBehindBall");
    else
        distanceBehindBall = 0.4;

    goToPos.setAvoidBall(properties->getBool("avoidBall"));
    goToPos.setCanGoOutsideField(properties->getBool("canGoOutsideField"));
}


Skill::Status BasicGoToPos::onUpdate() {

    if (! robot) return Status::Running;

    if (goToBall)
        targetPos = ball->pos;

    if (goBehindBall)
        targetPos = Coach::getPositionBehindBallToGoal(distanceBehindBall, false);


    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>((targetPos-robot->pos).angle());
    Vector2 velocity = goToPos.goToPos(robot, targetPos, control::GoToType::luTh);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);

    Vector2 deltaPos = targetPos - robot->pos;

    if (deltaPos.length() > errorMargin)
        return Status::Running;
    else
        return Status::Success;
}

}
}
