//
// Created by mrlukasbos on 4-12-18.
//

#ifndef ROBOTEAM_AI_DRAWER_H
#define ROBOTEAM_AI_DRAWER_H

#include <QtGui/QColor>
#include <roboteam_utils/Vector2.h>
#include <iostream>
#include <mutex>
#include <armadillo>

namespace rtt {
namespace ai {
namespace interface {

class Drawer {
    public:
        explicit Drawer() = default;
        static void setGoToPosLuThPoints(int id, std::vector<std::pair<Vector2, QColor>> points);
        static std::vector<std::pair<Vector2, QColor>> getGoToPosLuThPoints(int id);
        static void setKeeperPoints(int id, std::vector<std::pair<Vector2,QColor>> points);
        static std::vector<std::pair<Vector2, QColor>> getKeeperPoints(int id);
        static void setInterceptPoints(int id, std::vector<std::pair<Vector2,QColor>> points);
        static std::vector<std::pair<Vector2, QColor>> getInterceptPoints(int id);

        static void setVoronoiDiagram(arma::Mat<int> voronoiSegments, arma::Mat<float> voronoiNodes);
        static std::pair<arma::Mat<int>, arma::Mat<float>> getVoronoiDiagram(bool plot);

        static void setBezierCurve(std::vector<Vector2> curvePoints);
        static std::vector<Vector2> getBezierCurve(bool plot);

    private:
        static std::mutex goToPosMutex,keeperMutex,interceptMutex;
        static std::map<int, std::vector<std::pair<Vector2, QColor>>> GoToPosLuThPoints;
        static std::pair<arma::Mat<int>, arma::Mat<float>> voronoiDiagram;
        static std::vector<Vector2> bezierCurve;
        static std::map<int, std::vector<std::pair<Vector2, QColor>>> KeeperPoints;
        static std::map<int, std::vector<std::pair<Vector2, QColor>>> InterceptPoints;
};

} // interface
} // ai
} // rtt
#endif //ROBOTEAM_AI_DRAWER_H
