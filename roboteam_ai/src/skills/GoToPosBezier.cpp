//
// Created by selina on 11/14/18.
//

#include "GoToPosBezier.h"

namespace rtt {
namespace ai {

// TODO: Make if GrSim, if real robot statement

/// Init GoToPosBezier
void GoToPosBezier::onInitialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotID = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robotID, true)) {
            robot = *World::getRobotForId(robotID, true).get();
        }
        else {
            ROS_ERROR("GoToPos Initialize -> robot does not exist in world");
            currentProgress = Progression::INVALID;
            return;
        }
    }
    else {
        ROS_ERROR("GoToPos Initialize -> ROLE INVALID!!");
        currentProgress = Progression::INVALID;
        return;
    }

    if (properties->hasBool("goToBall")) {
        goToBall = properties->getBool("goToBall");
    }
    else goToBall = false;

    if (goToBall) {
        auto ball = World::getBall();
        targetPos = ball.pos;
    }
    else {
        if (properties->hasVector2("Position")) {
            Vector2 posVector = properties->getVector2("Position");
            targetPos = posVector;
            currentProgress = Progression::ON_THE_WAY;

        }
        else {
            ROS_ERROR("GoToPos Initialize -> No good X, Y or ROBOT_ID set in BB, GoToPos");
            currentProgress = Progression::FAIL;
        }
    }

    updateCurveData(0, false);

    // Set PID values
    pidBezier.setPID(3.0, 0.1, 0.1);
    pidStartTime = clock();
}

/// Get an update on the skill
bt::Node::Status GoToPosBezier::onUpdate() {
    // Update world data
    auto world = World::get_world();
    if (World::getRobotForId(robotID, true)) {
        robot = *World::getRobotForId(robotID, true).get();
    }
    else {
        ROS_ERROR("GoToPos Initialize -> robot does not exist in world");
        currentProgress = Progression::INVALID;
    }

    if (goToBall) {
        auto ball = World::getBall();
        targetPos = ball.pos;
    }

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return status::Failure;
    }

    // See where we are on the curve
    now = std::chrono::system_clock::now();
    timeDif = now - startTime;
    int currentPoint = (int) round((timeDif.count()/totalTime*curve.positions.size()));
    currentPoint = currentPoint >= (int) curve.positions.size() ? (int) curve.positions.size() - 1 : currentPoint;
    currentPoint = currentPoint < 0 ? 0 : currentPoint;

    // Adjust velocity using PID
    Vector2 posError = curve.positions[currentPoint] - robot.pos;
    Vector2 OutputPID = pidBezier.controlPIR2(posError, robot.vel);

    curve.velocities[currentPoint].x += OutputPID.x;
    curve.velocities[currentPoint].y += OutputPID.y;

    // Convert global to local
    double xVelocity =
            curve.velocities[currentPoint].x*cos(robot.angle) + curve.velocities[currentPoint].y*sin(robot.angle);
    double yVelocity =
            curve.velocities[currentPoint].x* -sin(robot.angle) + curve.velocities[currentPoint].y*cos(robot.angle);

    auto desiredAngle = (float)M_PI;

    // Send a move command
    sendMoveCommand(desiredAngle, xVelocity, yVelocity);

    // Determine if new curve is needed
    bool isAtEnd = currentPoint >= curve.positions.size() - 1;
    bool isErrorTooLarge = posError.length() > 0.5;
    bool hasTargetChanged = (pathFinder.getPath().back() - targetPos).length() > 0.5;

    // Calculate new curve if needed
    if (isAtEnd || isErrorTooLarge || hasTargetChanged || isAnyObstacleAtCurve(currentPoint)) {
        std::cerr << "------------------------" << std::endl;
        std::cerr << "       NEW CURVE \n Reason: ";
        if (isAtEnd) { std::cerr << "End of curve | "; }
        if (isErrorTooLarge) { std::cerr << "Error too large | "; }
        if (hasTargetChanged) { std::cerr << "Target changed | "; }
        if (isAnyObstacleAtCurve(currentPoint)) { std::cerr << "Obstacle on curve | "; }
        std::cerr << std::endl;
        std::cerr << "------------------------" << std::endl;

        updateCurveData(currentPoint, isErrorTooLarge);
    }

    // Now check the progress we made
    currentProgress = checkProgression();

    switch (currentProgress) {

        // Return the progression in terms of status
    case ON_THE_WAY:return status::Running;
    case DONE: return status::Success;
    case FAIL: return status::Failure;
    case INVALID: return status::Waiting;
    }

    return status::Failure;
}

/// Check if the vector is a valid one
bool GoToPosBezier::checkTargetPos(Vector2 pos) {
    // TODO: actually check
    return true;
}

/// Send a move robot command with a vector
void GoToPosBezier::sendMoveCommand(float desiredAngle, double xVelocity, double yVelocity) {
    if (! checkTargetPos(targetPos)) {
        ROS_ERROR("Target position is not correct GoToPos");
        return;
    }

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 0;
    command.w = 0;
    //command.w = (float) control::ControlUtils::calculateAngularVelocity(robot.angle, desiredAngle);

    command.x_vel = (float) xVelocity;
    command.y_vel = (float) yVelocity;

    publishRobotCommand(command);
}

/// Check the progress the robot made and alter the currentProgress
GoToPosBezier::Progression GoToPosBezier::checkProgression() {

    double dx = targetPos.x - robot.pos.x;
    double dy = targetPos.y - robot.pos.y;
    double deltaPos = (dx*dx) + (dy*dy);
    double maxMargin = 0.05;                 // max offset or something.

    if (abs(deltaPos) >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

GoToPosBezier::GoToPosBezier(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

std::string GoToPosBezier::node_name() {
    return "GoToPosBezier";
}

void GoToPosBezier::terminate(status s) {

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w = 0;

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

/// Create robot coordinates vector & other parameters for the path
void GoToPosBezier::updateCurveData(int currentPoint, bool isErrorTooLarge) {
    // TODO: don't hardcode end orientation & velocity
    // Update world data
    auto world = World::get_world();
    if (World::getRobotForId(robotID, true)) {
        robot = *World::getRobotForId(robotID, true).get();
    }
    else {
        ROS_ERROR("GoToPos Initialize -> robot does not exist in world");
        currentProgress = Progression::INVALID;
        return;
    }
    std::vector<Vector2> robotCoordinates;
    for (auto ourBot: world.us) {
        if (ourBot.id != robot.id) {
            robotCoordinates.emplace_back(ourBot.pos);
        }
    }
    for (auto theirBot: world.them) {
        robotCoordinates.emplace_back(theirBot.pos);
    }

    auto endAngle = (float) M_PI;
    float endVelocity = 0;
    Vector2 robotVel = robot.vel;
    auto startVelocity = (float) robotVel.length();
    Vector2 startPos = robot.pos;
    float startAngle = robot.angle;

    if (! curve.positions.empty() and ! isErrorTooLarge) {
        startPos = curve.positions[currentPoint];
        startAngle = curve.angles[currentPoint];
        startVelocity = (float) curve.velocities[currentPoint].length();
    }

    startAngle < 0 ? startAngle = startAngle + 2*(float) M_PI : startAngle;
    endAngle < 0 ? endAngle = endAngle + 2*(float) M_PI : endAngle;

    pathFinder.calculatePath(targetPos, startPos, endAngle, startAngle, startVelocity, endVelocity, robotCoordinates);

    /// Get path parameters
    curve.positions = pathFinder.getCurvePoints();
    curve.velocities = pathFinder.getVelocities();
    curve.angles = pathFinder.getAngles();
    totalTime = pathFinder.getTotalTime();

    /// Start timer
    startTime = std::chrono::system_clock::now();
}

bool GoToPosBezier::isAnyObstacleAtCurve(int currentPoint) {
    double margin = 2*constants::ROBOT_RADIUS;
    auto world = World::get_world();

    Vector2 robotPos;
    double maxCurveIndex =
            currentPoint + curve.positions.size()*(1.0/totalTime); // Foresee collision max 1.0 s in the future
    maxCurveIndex = maxCurveIndex > curve.positions.size() ? curve.positions.size() : maxCurveIndex;
    for (int i = currentPoint; i < maxCurveIndex; i ++) {
        for (auto ourBot: world.us) {
            if (ourBot.id != robot.id) {
                robotPos = ourBot.pos;
                if (robotPos.dist(curve.positions[i]) < margin) {
                    return true;
                }
            }
        }
        for (auto theirBot: world.them) {
            robotPos = theirBot.pos;
            if (robotPos.dist(curve.positions[i]) < margin) {
                return true;
            }
        }
    }
    return false;
}
} // ai
} // rtt
