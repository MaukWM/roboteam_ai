//
// Created by selina on 10/30/18.
//

#ifndef ROBOTEAM_ai_VORONOICREATOR_H
#define ROBOTEAM_ai_VORONOICREATOR_H

#include "roboteam_msgs/World.h"
#include <roboteam_msgs/GeometryData.h>
#include <armadillo>
#include "roboteam_utils/Vector2.h"
#include <cfloat>
#include "../../utilities/Field.h"

namespace rtt {
namespace ai {
class VoronoiCreator {
    private:
        // Functions

        static void lineFromPoints(std::pair<float, float> P, std::pair<float, float> Q, double &a, double &b,
                double &c);

        static void perpendicularBisectorFromLine(std::pair<float, float> P, std::pair<float, float> Q, double &a,
                double &b, double &c);

        static std::pair<float, float> lineLineIntersection(double a1, double b1, double c1, double a2, double b2,
                double c2);

        static arma::mat getIndexColumn(int a);

        static std::pair<arma::Mat<int>, arma::Mat<int>> startEndSegmentCreator(arma::Mat<int> triangleCombinations,
                arma::Mat<float> circleCenters, int startID, int endID);

        static arma::Mat<float> angleCalculator(int inp, arma::Mat<float> objectCoordidnates,
                arma::Mat<float> circleCenters,
                arma::Mat<int> voronoiSegments);

        static std::pair<std::pair<float, float>, std::pair<int, int>> orientationNodeCreator(int inp,
                arma::Mat<float> angles,
                float orientationAngle,
                arma::Mat<float> circleCenters, arma::Mat<float> objectCoordinates);

        static arma::Mat<float> removeIfInDefenceArea(arma::Mat<float> circleCenters, int startID, int endID);

        static arma::Mat<float> removeIfOutOfField(arma::Mat<float> circleCenters, int startID, int endID);

        static int findClosestPoint(std::pair<float, float> node, arma::Mat<float> circleCenters);


    public:
        static arma::Mat<int> possibleCombinations(arma::Mat<float> objectCoordinates);

        static std::pair<arma::Mat<float>, arma::Mat<float>> findCircumcircles(arma::Mat<int> triangleCombinations,
                arma::Mat<float> objectCoordinates);

        static std::pair<arma::Mat<float>, arma::Mat<int>> delaunayFilter(arma::Mat<float> objectCoordinates,
                arma::Mat<float> circleCenters, arma::Mat<float> radius, arma::Mat<int> triangleCombinations);

        static std::pair<arma::Mat<int>, arma::Mat<int>> findAdjacentCenter(arma::Mat<int> triangleCombinations);
        VoronoiCreator();
        // Struct
        struct parameters {
          arma::Mat<float> nodes;
          arma::Mat<int> segments;
          arma::Mat<int> triangles;
        };

        void voronoiMain(arma::Mat<float> objectCoordinates,
                float startOrientationAngle,
                float endOrientationAngle);

        parameters createVoronoi(arma::Mat<float> objectCoordinates,
                float startOrientationAngle, float endOrientationAngle, parameters voronoiParameters);

};
}
}

#endif //ROBOTEAM_ai_VORONOICREATOR_H

