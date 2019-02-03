//
// Created by simen on 03/02/19.
//

#include "GTPBezier.h"
#include "../utilities/Field.h"
#include "../control/ControlUtils.h"
#include "../interface/widget.h"

namespace rtt {
namespace ai {

GTPBezier::GTPBezier(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

/// Init the GoToPos skill
void GTPBezier::onInitialize() {
    goToBall = properties->getBool("goToBall");
    goBehindBall = properties->getBool("goBehindBall");
    if (properties->hasVector2("distanceBehindBall"))
        distanceBehindBall = properties->getDouble("distanceBehindBall");
    else
        distanceBehindBall = 0.5;
    if (properties->hasVector2("Position"))
        targetPos = properties->getVector2("Position");
    else {
        ROS_ERROR("GoToPos Initialize -> No good X or Y set in properties");
        currentProgress = Progression::FAIL;
    }
    if (properties->hasDouble("maxVel"))
        speed=properties->getDouble("maxVel");
    else
        speed=constants::DEFAULT_MAX_VEL;

    std::vector<Vector2> path;
    path.emplace_back(robot->pos);
    Vector2 botVel = robot->vel;
    path.emplace_back((Vector2)robot->pos + botVel.stretchToLength(1.5));
    path.emplace_back(targetPos);
    path.emplace_back(targetPos);
    std::vector<Vector2> robotCoordinates;
    robotCoordinates.emplace_back(Vector2(10,10));
    curveCreator.createCurve(path, robotCoordinates, 0, 0);
    curvePos = curveCreator.getCurvePositions();
    curveVel = curveCreator.getCurveVelocities();
    totalTime = curveCreator.getTotalTime();
    controller.setPID(3.0, 0.1, 0.1);
    interface::Drawer::setBezierCurve(curvePos);
    startTime = std::chrono::system_clock::now();
}

/// Get an update on the skill
bt::Node::Status GTPBezier::onUpdate() {
    if (! robot) return Status::Running;
    if (goToBall||goBehindBall) {
        if (! ball) return Status::Running;
    }
    if (goToBall) {
        targetPos = ball->pos;
    }

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }

    // See where we are on the curve
    now = std::chrono::system_clock::now();
    timeDif = now - startTime;
    int currentPoint = (int) round((timeDif.count()/totalTime*curvePos.size()));
    currentPoint = currentPoint >= (int) curvePos.size() ? (int) curvePos.size() - 1 : currentPoint;
    currentPoint = currentPoint < 0 ? 0 : currentPoint;

    // Adjust velocity using PID
    Vector2 posError = curvePos[currentPoint] - robot->pos;
    Vector2 OutputPID = controller.controlPIR(posError, robot->vel);

    curveVel[currentPoint].x += OutputPID.x;
    curveVel[currentPoint].y += OutputPID.y;

    // Convert global to local
    double xVelocity =
            curveVel[currentPoint].x*cos(robot->angle) + curveVel[currentPoint].y*sin(robot->angle);
    double yVelocity =
            curveVel[currentPoint].x*- sin(robot->angle) + curveVel[currentPoint].y*cos(robot->angle);

//    Vector2 botPos = robot->pos;
//    Vector2 refVel = (curvePos[currentPoint] - botPos).stretchToLength(speed);
//    float xVelocity = refVel.x;
//    float yVelocity = refVel.y;

    sendMoveCommand(xVelocity, yVelocity);

    // Now check the progress we made
    deltaPos = targetPos - robot->pos;
    currentProgress = checkProgression();

    switch (currentProgress) {

        // Return the progression in terms of Status
    case ON_THE_WAY:
        return Status::Running;
    case DONE:
        return Status::Success;
    case FAIL:
        return Status::Failure;
    }

    return Status::Failure;
}

void GTPBezier::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

/// Send a move robot command with a vector
void GTPBezier::sendMoveCommand(float xVelocity, float yVelocity) {

    // TODO: get correct kp from 20-sim model
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 0;

    command.w = 0;//atan(yVelocity/xVelocity);
    command.x_vel = xVelocity;
    command.y_vel = yVelocity;
    publishRobotCommand(command);
    //commandSend = true;
}

/// Send a move robot command with a vector
void GTPBezier::sendMoveCommand2() {
    // TODO: get correct kp from 20-sim model
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;

    command.w = static_cast<float>(deltaPos.angle());
    Vector2 deltaPosUnit = deltaPos.normalize();

    command.x_vel = static_cast<float>(deltaPosUnit.x*speed);// abs(angularVel)/(abs(angularVel)-1);
    command.y_vel = static_cast<float>(deltaPosUnit.y*speed);
    publishRobotCommand(command);
    commandSend = true;
}

/// Check the progress the robot made a9nd alter the currentProgress
GTPBezier::Progression GTPBezier::checkProgression() {
    double maxMargin = 0.2;                        // max offset or something.
    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

} // ai
} // rtt