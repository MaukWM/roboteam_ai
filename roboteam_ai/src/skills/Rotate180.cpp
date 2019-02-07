//
// Created by baris on 24/10/18.
//

#include "Rotate180.h"
#include "../utilities/Field.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

Rotate180::Rotate180(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

/// Init the Rotate180 skill
void Rotate180::onInitialize() {
    timer = std::chrono::system_clock::now();
    desiredAngle = 0;
}

/// Get an update on the skill
bt::Node::Status Rotate180::onUpdate() {
    now = std::chrono::system_clock::now();
    std::chrono::duration<double> timeDiff = now - timer;
    if (timeDiff.count() < 3) {
        desiredAngle = 0;
    } else if (timeDiff.count() < 6) {
        desiredAngle = (float)M_PI;
    } else {
        timer = std::chrono::system_clock::now();
    }
    
    // Send a move command
    sendMoveCommand();
    
    return Status::Running;
}

void Rotate180::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = 0;//static_cast<float>(deltaPos.angle());

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

/// Send a move robot command with a vector
void Rotate180::sendMoveCommand() {

    // TODO: get correct kp from 20-sim model
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;

    command.w = desiredAngle;

    command.x_vel = 0;// abs(angularVel)/(abs(angularVel)-1);
    command.y_vel = 0;
    publishRobotCommand(command);
    //commandSend = true;
}
} // ai
} // rtt