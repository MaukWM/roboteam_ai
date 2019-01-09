//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
//std::map<int, std::vector<Vector2>> Drawer::GoToPosLuThPoints;
std::pair<arma::Mat<int>, arma::Mat<float>> Drawer::voronoiDiagram;
std::vector<Vector2> Drawer::bezierCurve;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::GoToPosLuThPoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::KeeperPoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::InterceptPoints;

std::mutex Drawer::keeperMutex;
std::mutex Drawer::goToPosMutex;
std::mutex Drawer::interceptMutex;

// Setters
void Drawer::setGoToPosLuThPoints(int id, std::vector<std::pair<rtt::Vector2, QColor>> points) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    std::pair<int, std::vector<std::pair<rtt::Vector2, QColor>>> pair{id, std::move(points)};

    GoToPosLuThPoints.erase(id);
    GoToPosLuThPoints.insert(pair);
}

void Drawer::setVoronoiDiagram(arma::Mat<int> voronoiSegments, arma::Mat<float> voronoiNodes) {
    voronoiDiagram = std::make_pair(voronoiSegments, voronoiNodes);
}

void Drawer::setBezierCurve(std::vector<Vector2> curvePoints) {
    bezierCurve = curvePoints;
}

// Getters
std::vector<std::pair<Vector2, QColor>> Drawer::getGoToPosLuThPoints(int id) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    if (GoToPosLuThPoints.find(id) != GoToPosLuThPoints.end()) {
        return GoToPosLuThPoints.at(id);
    }
    return {};
}

std::pair<arma::Mat<int>, arma::Mat<float>> Drawer::getVoronoiDiagram(bool plot) {

    if (plot) {
        return voronoiDiagram;
    }
    return {};
}

std::vector<Vector2> Drawer::getBezierCurve(bool plot) {

    if (plot) {
        return bezierCurve;
    }
    return {};
}

void Drawer::setKeeperPoints(int id,std::vector<std::pair<rtt::Vector2, QColor>> points) {
    std::lock_guard<std::mutex> lock(keeperMutex);

    std::pair<int, std::vector<std::pair<rtt::Vector2, QColor>>> pair{id, std::move(points)};

    KeeperPoints.erase(id);
    KeeperPoints.insert(pair);
}
std::vector<std::pair<Vector2, QColor>> Drawer::getKeeperPoints(int id) {
    std::lock_guard<std::mutex> lock(keeperMutex);

    if (KeeperPoints.find(id) != KeeperPoints.end()) {
        return KeeperPoints.at(id);
    }
    return {};

}
void Drawer::setInterceptPoints(int id,std::vector<std::pair<rtt::Vector2, QColor>> points) {
    std::lock_guard<std::mutex> lock(interceptMutex);

    std::pair<int, std::vector<std::pair<rtt::Vector2, QColor>>> pair{id, std::move(points)};

    InterceptPoints.erase(id);
    InterceptPoints.insert(pair);
}
std::vector<std::pair<Vector2, QColor>> Drawer::getInterceptPoints(int id) {
    std::lock_guard<std::mutex> lock(interceptMutex);

    if (InterceptPoints.find(id) != InterceptPoints.end()) {
        return InterceptPoints.at(id);
    }
    return {};

}
} // interface
} // ai
} // rtt