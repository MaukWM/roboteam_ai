//
// Created by baris on 9-1-19.
//

#ifndef ROBOTEAM_AI_VORONOIDATA_H
#define ROBOTEAM_AI_VORONOIDATA_H

#include "VoronoiCreator.h"
namespace rtt {
namespace ai {
class VoronoiData {
    public:
        using bezier = rtt::ai::VoronoiCreator::parameters;
        static void bezierMain();
        static bezier getData();


    private:
        static bezier currentData;
        void setData(bezier newData);
        static std::mutex lockie;

};
}
}
#endif //ROBOTEAM_AI_VORONOIDATA_H
