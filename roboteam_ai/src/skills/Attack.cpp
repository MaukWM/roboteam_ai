//
// Created by thijs on 17-12-18.
//

#include "Attack.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

// TODO: WTF HARDCODED SHIT EVERYWHERE
/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;
    Vector2 ball = World::getBall()->pos;
    Vector2 behindBall = Coach::getPositionBehindBallToGoal(0.5, false);
    Vector2 deltaBall = behindBall - ball;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;


    GoToType goToType;

    if (! Coach::isRobotBehindBallToGoal(0.3, false, robot->pos)) {
        goToPos.setAvoidBall(true);
        targetPos = behindBall;
        command.use_angle = 1;
        command.w = static_cast<float>((ball - (Vector2) (robot->pos)).angle());
        goToType = GoToType::luTh;
    }
    else {
        targetPos = ball;
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) {- 1.0, - 1.0}*deltaBall).angle());
        if (Coach::doesRobotHaveBall(robot->id, true, rtt::ai::constants::MAX_BALL_RANGE)) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::constants::MAX_KICK_POWER);
            command.kicker_forced = 1;
        }
        goToType = GoToType::basic;
    }
    Vector2 velocity;
    velocity = goToPos.goToPos(robot, targetPos, goToType);

    velocity = control::ControlUtils::VelocityLimiter(velocity);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);

    return Status::Running;
}

void Attack::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());
    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

} // ai
} // rtt