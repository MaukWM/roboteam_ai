#include <utility>

//
// Created by baris on 9-1-19.
//

#include "VoronoiData.h"

namespace rtt {
namespace ai {

void VoronoiData::bezierMain() {

}

VoronoiData::bezier VoronoiData::getData() {
    std::lock_guard<std::mutex> lock(lockie);
    return currentData;
}

void VoronoiData::setData(VoronoiData::bezier newData) {
    std::lock_guard<std::mutex> lock(lockie);
    currentData = std::move(newData);

}

}
}

