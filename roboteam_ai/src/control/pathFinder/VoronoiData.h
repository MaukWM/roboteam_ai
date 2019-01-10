//
// Created by baris on 9-1-19.
//

#ifndef ROBOTEAM_AI_VORONOIDATA_H
#define ROBOTEAM_AI_VORONOIDATA_H

#include "VoronoiCreator.h"
namespace rtt {
namespace ai {
class VoronoiData {
        using bezier = rtt::ai::VoronoiCreator::parameters;

    public:
        static void bezierMain();
        static bezier getData();
        static roboteam_msgs::World getLastWorld();


    private:
        static bezier currentData;
        static roboteam_msgs::World lastWorld;
        static void setData(bezier newData, roboteam_msgs::World world);
        static std::mutex loki;
        static std::mutex worldLock;

        static arma::Mat<float> makeMatrix(roboteam_msgs::World world);
};
}
}
#endif //ROBOTEAM_AI_VORONOIDATA_H
