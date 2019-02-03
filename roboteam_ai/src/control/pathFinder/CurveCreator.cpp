//
// Created by simen on 06/11/18.
//

#include "CurveCreator.h"

namespace rtt {
namespace ai {
CurveCreator::CurveCreator() {
    this->numPoints = 1000;
}

CurveCreator::CurveCreator(float numPoints) {
    this->numPoints = numPoints;
}

/// Main function, calls functions to create a curve
void CurveCreator::createCurve(std::vector<Vector2> path, std::vector<Vector2> robotCoordinates,
        float startVelocity, float endVelocity) {
    pathNodes = path;
//    robotCoordinates.erase(robotCoordinates.begin()); // Delete start point from objects
//    robotCoordinates.erase(robotCoordinates.begin()); // Delete end point from objects
    objectCoordinates = robotCoordinates;

    if (pathNodes.size() < 2) {
        std::cout << "You need to enter at least 2 nodes in order to create a curve." << std::endl;
    }
    else {
        /// Determine locations of control points
        //calculateControlPoints();
        controlPoints = path;
//        bool isEndPiece = controlPoints.back() == pathNodes.back();
//        shiftVelocityControlPoints(startVelocity, endVelocity, isEndPiece, false);

        /// Limit velocity and acceleration by stretching time
        calculateVelocity();
        calculateAcceleration();
        calculateTotalTime();
        scaleVelocities();
        scaleAcceleration();
//
//        /// Shift control points again to have the right velocities
//        shiftVelocityControlPoints(startVelocity, endVelocity, isEndPiece, true);
//
//        /// Calculate all needed variables
//        calculateVelocity();
//        calculateAcceleration();
//        scaleVelocities();
//        scaleAcceleration();
        calculateOrientation();
        convertPointsToCurve();
    }
}

/// Calculate the position where the control points of the curve should be
void CurveCreator::calculateControlPoints() {
    controlPoints.push_back(pathNodes[0]); // First path node is always a control point
    controlPoints.push_back(pathNodes[1]); // Second path node is always a control point

    if (pathNodes.size() > 2) {
        controlPoints.push_back(pathNodes[2]); // Third path node is always a control point if it exists
        std::vector<Vector2> convex;
        std::vector<Vector2> dangerousObstacle;

        for (int i = 2; i < pathNodes.size() - 1; i ++) {
            convex = createConvexHull();
            dangerousObstacle = findDangerousObstacle(convex);
            if (dangerousObstacle.empty()) {
                controlPoints.push_back(pathNodes[i] + (pathNodes[i + 1] - pathNodes[i]).scale(
                        1)); // TODO: .scale() could be used to minimize curvature
            }
            else {
                // change control point until no obstacle is in the convex anymore
                // TODO: determine the control point in a mathematical way
                int count = 0;
                while (! dangerousObstacle.empty() and count < 10) {
                    controlPoints[controlPoints.size() - 1] = controlPoints[controlPoints.size() - 2] +
                            (controlPoints[controlPoints.size() - 1] - controlPoints[controlPoints.size() - 2]).scale(
                                    0.5);
                    convex = createConvexHull();
                    dangerousObstacle = findDangerousObstacle(convex);
                    count ++;
                }
                controlPoints[controlPoints.size() - 1] = controlPoints[controlPoints.size() - 2] +
                        (controlPoints[controlPoints.size() - 1] - controlPoints[controlPoints.size() - 2]).scale(
                                1); // TODO: .scale() could be used to minimize curvature
                break;
            }
        }

//        // Add velocity control points (correct placing is done later by shiftVelocityControlPoints)
//        Vector2 startVelCP = controlPoints[0] + (controlPoints[1] - controlPoints[0]).scale(0.5);
//        Vector2 endVelCP = controlPoints[controlPoints.size() - 2]
//                + (controlPoints[controlPoints.size() - 1] - controlPoints[controlPoints.size() - 2]).scale(0.5);
//
//        controlPoints.insert(controlPoints.begin() + 1, startVelCP);
//        controlPoints.insert(controlPoints.end() - 1, endVelCP);
    }
}

/// Find dangerous obstacle in control point convex hull, can be empty
std::vector<Vector2> CurveCreator::findDangerousObstacle(std::vector<Vector2> convex) {
    std::vector<Vector2> dangerousObstacle;
    for (Vector2 &obstaclePos: objectCoordinates) {
        if (isObstacleInConvexHull(convex, obstaclePos)) {
            dangerousObstacle.push_back(obstaclePos);
            break;
        }
    }
    return dangerousObstacle;
}

/// Use Graham Scan to turn curve piece into Convex Hull
std::vector<Vector2> CurveCreator::createConvexHull() {
    // Find point P that has the lowest y-coordinate (or x-coordinate if y is equal)
    Vector2 P(DBL_MAX, DBL_MAX);
    for (Vector2 &point : controlPoints) {
        if ((point.y < P.y) or (point.y == P.y and point.x < P.x)) {
            P = point;
        }
    }

    // Sort vector based on angle between P -> point and x-axis
    std::vector<Vector2> sortedList;
    sortedList.push_back(P);

    Vector2 unitVecX(1, 0);
    for (Vector2 &point : controlPoints) {
        if (point != P) {
            if (sortedList.size() == 1) {
                sortedList.push_back(point);
            }
            else {
                for (int i = 1; i < sortedList.size(); i ++) {
                    if ((point - P).normalize().dot(unitVecX) > (sortedList[i] - P).normalize().dot(unitVecX)) {
                        sortedList.insert(sortedList.begin() + i, point);
                        break;
                    }
                    else if (i == sortedList.size() - 1) {
                        sortedList.push_back(point);
                        break;
                    }
                }
            }
        }
    }

    // Iterate over points and check whether making a left or right turn
    std::vector<Vector2> convex;
    convex.push_back(sortedList[0]);
    convex.push_back(sortedList[1]);

    double crossProduct;
    int index = 0;
    Vector2 vec1, vec2;
    for (int i = 2; i < sortedList.size(); i ++) {
        vec1 = convex[index + 1] - convex[index];
        vec2 = sortedList[i] - convex[index];
        crossProduct = vec1.x*vec2.y - vec1.y*vec2.x;

        if (crossProduct < 0) {
            // Right turn, erase last point from convex and add next one from list
            convex.pop_back();
            convex.push_back(sortedList[i]);
        }
        else {
            // Left turn, add point to convex
            convex.push_back(sortedList[i]);
            index ++;
        }
    }

    return convex;
}

/// Check if an obstacle is in the convex hull
bool CurveCreator::isObstacleInConvexHull(std::vector<Vector2> convex, Vector2 obstPos) {
    // Check if the obstacle is in the convex
    float sumOfAngles = 0;
    // If the sum of angles between obstacle to vertices is 2*PI, the obstacle center is inside the polygon.
    // Simultaneously, check if the obstacle is on one of the edges.
    convex.push_back(convex[0]); // Add first point to close convex at the end
    for (int i = 0; i < convex.size() - 1; i ++) {
        if (convex[i] != convex[i + 1]) {
            sumOfAngles += acos((convex[i] - obstPos).normalize().dot((convex[i + 1] - obstPos).normalize()));
            // Is obstacle on edge?
            if (distancePointToLine(obstPos, convex[i], convex[i + 1]) < robotDiameter/2) {
                return true;
            }
        }
    }
    return abs(sumOfAngles - 2*M_PI) < 0.002*M_PI; // allow 0.1 percent deviation
}

/// Calculate the distance from a point to a line segment between the two end points
float
CurveCreator::distancePointToLine(Vector2 point, Vector2 linepoint1, Vector2 linepoint2) {
    // use vector formulation
    // line: x = a + t*n, where a = linepoint1
    // point: p
    Vector2 n = (linepoint2 - linepoint1).normalize();
    double t = n.dot(point - linepoint1);
    if (t < 0) {
        // linepoint1 is closest to point
        return (float) point.dist(linepoint1);
    }
    else if (t > (linepoint2 - linepoint1).length()) {
        // linepoint 2 is closest to point
        return (float) point.dist(linepoint2);
    }
    else {
        // closest point is on the line segment
        return (float) (linepoint1 - point + n.stretchToLength(t)).length();
    }
}

/// Shift control points to the set such that the initial and final velocity demands have been met
void CurveCreator::shiftVelocityControlPoints(float startVelocity, float endVelocity, bool isEndPiece,
        bool includeTime) {
    auto numControlPoints = controlPoints.size() + 1; // include new start velocity control point

    float startScaleFactor = startVelocity/(numControlPoints - 1);
    startScaleFactor = includeTime ? startScaleFactor*totalTime : startScaleFactor;

    if (startScaleFactor < 0) {
        std::cout << "startScaleFactor is smaller than 0" << std::endl;
    }

    startScaleFactor = startScaleFactor > (float) (pathNodes[1] - pathNodes[0]).length()
                       ? (float) (pathNodes[1] - pathNodes[0]).length() : startScaleFactor;

    Vector2 startVelCP = pathNodes[0] + (pathNodes[1] - pathNodes[0]).stretchToLength(startScaleFactor);
    controlPoints.insert(controlPoints.begin() + 1, startVelCP);

    if (isEndPiece) {
        float endScaleFactor = endVelocity/(numControlPoints - 1);
        endScaleFactor = includeTime ? endScaleFactor*totalTime : endScaleFactor;

        endScaleFactor = endScaleFactor > (float) (pathNodes[pathNodes.size() - 2]
                - pathNodes[pathNodes.size() - 1]).length()
                         ? (float) (pathNodes[pathNodes.size() - 2]
                        - pathNodes[pathNodes.size() - 1]).length() : endScaleFactor;

        Vector2 endVelCP = pathNodes.back()
                + (pathNodes[pathNodes.size() - 2] - pathNodes[pathNodes.size() - 1]).stretchToLength(
                        endScaleFactor);

        controlPoints[controlPoints.size() - 2] = endVelCP;
    }
}

/// Convert the control points that have been calculated to a set of curve points
void CurveCreator::convertPointsToCurve() {
    float curveDegree = controlPoints.size() - 1;

    if (curvePositions.empty()) {
        for (int i = 0; i < numPoints; i ++) {
            curvePositions.emplace_back(Vector2(0, 0));
        }
    } else {
        for (int i = 0; i < numPoints; i ++) {
            curvePositions[i] = Vector2(0, 0);
        }
    }


    double coefficient;
    double controlPointWeight;
    for (int i = 0; i <= curveDegree; i ++) {
        controlPointWeight = factorial(curveDegree)/(factorial(i)*factorial(curveDegree - i));
        float t = 0; // curve parameter
        for (int j = 0; j < numPoints; j ++) {
            coefficient = controlPointWeight*pow(t, i)*pow(1 - t, curveDegree - i);
            curvePositions[j] = curvePositions[j] + controlPoints[i].scale(coefficient);
            t += 1/numPoints;
        }
    }
}

/// Calculate the curve velocity at each of the curve points
void CurveCreator::calculateVelocity() {
    float curveDegree = controlPoints.size() - 1;
    double coefficient;
    double controlPointWeight; // weight that differs per control point
    float t; // curve parameter

    curveVelocities = {}; // empty vector since this function is used twice
    for (int i = 0; i < numPoints; i ++) {
        curveVelocities.emplace_back(Vector2(0, 0));
    }
    for (int i = 0; i <= curveDegree; i ++) {
        controlPointWeight = factorial(curveDegree)/(factorial(i)*factorial(curveDegree - i));
        t = 0;
        for (int j = 0; j < numPoints; j ++) {
            coefficient = controlPointWeight*(i*pow(t, i - 1)*pow(1 - t, curveDegree - i)
                    - pow(t, i)*(curveDegree - i)*pow(1 - t, curveDegree - i - 1));
            curveVelocities[j] = curveVelocities[j] + controlPoints[i].scale(coefficient);
            t += 1/numPoints;
        }
    }
    // Fix NAN data by extrapolation
    curveVelocities[0] = curveVelocities[1] + (curveVelocities[1] - curveVelocities[2]);
    curveVelocities.back() = curveVelocities[curveVelocities.size() - 2]
            + (curveVelocities[curveVelocities.size() - 2] - curveVelocities[curveVelocities.size() - 3]);
}

/// Calculate the curve acceleration at each of the curve points
void CurveCreator::calculateAcceleration() {
    float curveDegree = controlPoints.size() - 1;

    for (int i = 0; i < numPoints; i ++) {
        curveAccelerations.emplace_back(Vector2(0, 0));
    }

    double coefficient;
    double controlPointWeight;
    for (int i = 0; i <= curveDegree; i ++) {
        controlPointWeight = factorial(curveDegree)/(factorial(i)*factorial(curveDegree - i));
        float t = 0; // curve parameter
        for (int j = 0; j < numPoints; j ++) {
            coefficient = controlPointWeight*(i*(i - 1)*pow(t, i - 2)*pow(1 - t, curveDegree - i)
                    - 2*i*pow(t, i - 1)*(curveDegree - i)*pow(1 - t, curveDegree - i - 1)
                    + pow(t, i)*(curveDegree - i)*(curveDegree - i - 1)*pow(1 - t, curveDegree - i - 2));
            curveAccelerations[j] = curveAccelerations[j] + controlPoints[i].scale(coefficient);
            t += 1/numPoints;
        }
    }

    // Fix NAN data by extrapolation
    curveAccelerations[0] = curveAccelerations[1] + (curveAccelerations[1] - curveAccelerations[2]);
    curveAccelerations.back() = curveAccelerations[curveAccelerations.size() - 2]
            + (curveAccelerations[curveAccelerations.size() - 2] - curveAccelerations[curveAccelerations.size() - 3]);
}

/// Calculate the total time in which the curve can be finished based on maximum velocity
void CurveCreator::calculateTotalTime() {
    // Get the highest velocity that will be reached in the curve
    float highestVelocity = 0.0;
    for (const Vector2 &vel : curveVelocities) {
        highestVelocity = ((float) vel.length() > highestVelocity) ? (float) vel.length() : highestVelocity;
    }
    float velocityFactor = highestVelocity/maxVelocity; // multiply velocity by this factor to limit it

    // Get the highest acceleration that will be reached in the curve
    float highestAcceleration = 0.0;
    for (const Vector2 &acc : curveAccelerations) {
        highestAcceleration = ((float) acc.length() > highestAcceleration) ? (float) acc.length() : highestAcceleration;
    }
    auto accelerationFactor = (float) sqrt(
            highestAcceleration/maxAcceleration); // multiply acceleration by this factor to limit it

    totalTime = velocityFactor > accelerationFactor ? velocityFactor
                                                    : accelerationFactor; // Take the greater one to limit both
}

/// Scale velocity to maximum
void CurveCreator::scaleVelocities() {
    for (int i = 0; i < curveVelocities.size(); i ++) {
        curveVelocities[i].x /= totalTime;
        curveVelocities[i].y /= totalTime;
    }
}

/// Scale acceleration to maximum
void CurveCreator::scaleAcceleration() {
    for (int i = 0; i < curveAccelerations.size(); i ++) {
        curveAccelerations[i].x /= totalTime*totalTime;
        curveAccelerations[i].y /= totalTime*totalTime;
    }
}

/// Calculate the orientation at each of the curve points
void CurveCreator::calculateOrientation() {
    for (Vector2 &vel: curveVelocities) {
        curveOrientations.push_back((float) vel.angle());
    }

    // Fix first point by extrapolation
    curveOrientations[0] = curveOrientations[1] - (curveOrientations[2] - curveOrientations[1]);
}

/// Factorial function: result = x!
double CurveCreator::factorial(float x) {
    double result = 1;
    for (int i = 1; i <= x; i += 1) {
        result *= i;
    }
    return result;
}

/// GETTERS
const std::vector<Vector2> &CurveCreator::getCurvePositions() const {
    return curvePositions;
}

const std::vector<Vector2> &CurveCreator::getCurveVelocities() const {
    return curveVelocities;
}

const std::vector<float> &CurveCreator::getCurveOrientations() const {
    return curveOrientations;
}

float CurveCreator::getTotalTime() const {
    return totalTime;
}
} // ai
} // rtt