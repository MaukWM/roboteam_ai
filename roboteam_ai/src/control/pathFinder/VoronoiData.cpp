#include <utility>

#include <roboteam_ai/src/utilities/World.h>

//
// Created by baris on 9-1-19.
//

#include "VoronoiData.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
namespace rtt {
namespace ai {
using bezier = rtt::ai::VoronoiCreator::parameters;

bezier VoronoiData::currentData;
std::mutex VoronoiData::loki;
std::mutex VoronoiData::worldLock;
roboteam_msgs::World VoronoiData::lastWorld;

void VoronoiData::bezierMain() {

    while (true) {

        // get the world data
        auto world = World::get_world();
        auto objectCoordinates = VoronoiData::makeMatrix(world);

        // calculate
        // Calculate all possible triangle combinations
        arma::Mat<int> triangleCombinations = VoronoiCreator::possibleCombinations(objectCoordinates);

        // Calculate the radius and center of each triangle
        std::pair<arma::Mat<float>, arma::Mat<float >> circleParameters = VoronoiCreator::findCircumcircles(
                triangleCombinations,
                objectCoordinates);

        // Make triangles Delaunay
        std::pair<arma::Mat<float>, arma::Mat<int >> delaunayTriangles = VoronoiCreator::delaunayFilter(
                objectCoordinates,
                circleParameters.first, circleParameters.second, triangleCombinations);
        arma::Mat<float> circleCenters = delaunayTriangles.first;
        triangleCombinations = delaunayTriangles.second;

        // Find triangles that share a side
        // First = triangles, second = centers
        std::pair<arma::Mat<int>, arma::Mat<int >> adjacent = VoronoiCreator::findAdjacentCenter(triangleCombinations);

        VoronoiCreator::parameters voronoiParameters;
        voronoiParameters.nodes = circleCenters;
        voronoiParameters.segments = adjacent.second;
        voronoiParameters.triangles = triangleCombinations;
        voronoiParameters.objects = objectCoordinates;

        // set
        //locks
        setData(voronoiParameters, world);

        // TODO make money, get bitchez

    }

}

VoronoiData::bezier VoronoiData::getData() {

    std::lock_guard<std::mutex> lock(loki);
    return currentData;
}

// Internal use
void VoronoiData::setData(VoronoiData::bezier newData, roboteam_msgs::World world) {
    std::lock_guard<std::mutex> lock(loki);
    currentData = std::move(newData);
    std::lock_guard<std::mutex> wLock(worldLock);
    lastWorld = std::move(world);
}
arma::Mat<float> VoronoiData::makeMatrix(const roboteam_msgs::World world) {

    std::vector<Vector2> robotCoordinates;
    for (auto ourBot: world.us) {
        robotCoordinates.emplace_back(ourBot.pos);
    }
    for (auto theirBot: world.them) {
        robotCoordinates.emplace_back(theirBot.pos);
    }

    auto ball = World::getBall();
    auto endPosition = ball.pos;

    // Add start & end position to objects
    std::vector<Vector2> objectCoordinatesVector;
    objectCoordinatesVector.emplace_back(endPosition);

    float safetyMargin = 0.1; // m TODO from parameter list; distance between field and field border
    int nSteps = 5; // determines amount of safety points

    float fieldWidth = Field::get_field().field_width;
    float fieldLength = Field::get_field().field_length;

    std::vector<float> xEdges = {- fieldWidth/2 - safetyMargin, fieldWidth/2 + safetyMargin};
    for (float x: xEdges) {
        float y;
        for (int i = 0; i < nSteps; i ++) {
            y = i*fieldLength/nSteps - fieldLength/2;
            objectCoordinatesVector.emplace_back(Vector2(x, y));
        }
    }

    std::vector<float> yEdges = {- fieldLength/2 - safetyMargin, fieldLength/2 + safetyMargin};
    for (float y: yEdges) {
        float x;
        for (int i = 0; i < nSteps; i ++) {
            x = i*fieldWidth/nSteps - fieldWidth/2;
            objectCoordinatesVector.emplace_back(Vector2(x, y));
        }
    }

    // Add robot coordinates
    objectCoordinatesVector.insert(objectCoordinatesVector.end(), robotCoordinates.begin(), robotCoordinates.end());

    // Change object vector to matrix
    arma::Mat<float> temp;
    arma::Mat<float> objectCoordinatesMatrix;
    for (auto i = (int) objectCoordinatesVector.size() - 1; i > - 1; i --) {
        temp << objectCoordinatesVector[i].x << objectCoordinatesVector[i].y << arma::endr;
        objectCoordinatesMatrix.insert_rows(0, temp);
    }

    std::cout << objectCoordinatesMatrix << std::endl;

    return objectCoordinatesMatrix;
}
roboteam_msgs::World VoronoiData::getLastWorld() {
    std::lock_guard<std::mutex> lock(worldLock);
    return lastWorld;
}

}
}

#pragma clang diagnostic pop