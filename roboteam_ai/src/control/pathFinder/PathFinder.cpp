//
// Created by selina on 10/31/18.
//

#include "PathFinder.h"
#include <time.h>

namespace rtt {
namespace ai {
PathFinder::PathFinder() {
}

void PathFinder::calculatePath(Vector2 endPosition, Vector2 startPosition, float endAngle, float startAngle,
        float startVelocity,
        float endVelocity, std::vector<Vector2> robotCoordinates) {

    // Set start & end ID: always these values
    int startID = 0;
    int endID = 1;

    // Add start & end position to objects
    std::vector<Vector2> objectCoordinatesVector;
    objectCoordinatesVector.emplace_back(startPosition);
    objectCoordinatesVector.emplace_back(endPosition);

    // Add safety coordinates
    float safetyMargin = 0.1; // m TODO from parameter list; distance between field and field border
    int nSteps = 5; // determines amount of safety points

    float fieldWidth = Field::get_field().field_width;
    float fieldLength = Field::get_field().field_length;

    std::vector<float> xEdges = {- fieldWidth/2 - safetyMargin, fieldWidth/2 + safetyMargin};
    for (float x: xEdges) {
        float y;
        for (int i = 0; i < nSteps; i++) {
            y = i * fieldLength/nSteps - fieldLength/2;
            objectCoordinatesVector.emplace_back(Vector2(x, y));
        }
    }

    std::vector<float> yEdges = {- fieldLength/2 - safetyMargin, fieldLength/2 + safetyMargin};
    for (float y: yEdges) {
        float x;
        for (int i = 0; i < nSteps; i++) {
            x = i * fieldWidth/nSteps - fieldWidth/2;
            objectCoordinatesVector.emplace_back(Vector2(x, y));
        }
    }

    // Add robot coordinates
    objectCoordinatesVector.insert(objectCoordinatesVector.end(), robotCoordinates.begin(), robotCoordinates.end());

    // Change object vector to matrix
    arma::Mat<float> temp;
    arma::Mat<float> objectCoordinatesMatrix;
    for (auto i = objectCoordinatesVector.size() - 1; i > - 1; i --) {
        temp << objectCoordinatesVector[i].x << objectCoordinatesVector[i].y << arma::endr;
        objectCoordinatesMatrix.insert_rows(0, temp);
    }

    // Create Voronoi diagram
    VoronoiCreator voronoiCreator;
    VoronoiCreator::parameters voronoiParameters = voronoiCreator.createVoronoi(objectCoordinatesMatrix,
            startAngle, endAngle);

    // Find the shortest path through the diagram
    FindShortestPath shortestPathFinder;
    path = shortestPathFinder.calculateShortestPath(voronoiParameters.nodes,
            voronoiParameters.segments, startID, endID, objectCoordinatesVector, startAngle, endAngle);

    // Create a curve across this path
    CurveCreator curveCreator;
    curveCreator.createCurve(path, objectCoordinatesVector, startVelocity, endVelocity);

    // Set the variables
    curvePoints = curveCreator.getCurvePositions();
    velocities = curveCreator.getCurveVelocities();
    angles = curveCreator.getCurveOrientations();
    totalTime = curveCreator.getTotalTime();

    // Update the interface with Bezier
    interface::Drawer::setVoronoiDiagram(voronoiParameters.segments, voronoiParameters.nodes);
    interface::Drawer::setBezierCurve(curvePoints);
}

std::vector<Vector2> PathFinder::getPath() {
    return path;
}

std::vector<Vector2> PathFinder::getCurvePoints() {
    return curvePoints;
}

std::vector<Vector2> PathFinder::getVelocities() {
    return velocities;
}

std::vector<float> PathFinder::getAngles() {
    return angles;
}
float PathFinder::getTotalTime() const {
    return totalTime;
}
} //ai
} //rtt