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


    private:
        static bezier currentData;
        static void setData(bezier newData);
        static std::mutex lockie;
        static arma::Mat<float> makeMatrix();
};
}
}
#endif //ROBOTEAM_AI_VORONOIDATA_H
