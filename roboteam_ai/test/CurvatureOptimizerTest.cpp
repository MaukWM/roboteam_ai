//
// Created by simen on 20/12/18.
//

#include "../src/control/pathFinder/CurvatureOptimizer.h"
#include "gtest/gtest.h"

namespace rtt {
namespace ai {
TEST(CurvatureOptimizerTest, CurvatureOptimizerTest) {
    /// Make test data
    std::vector<Vector2> controlPoints = {Vector2(0,0), Vector2(1,2), Vector2(2,2), Vector2(3,0), Vector2(5,0)};
    std::vector<Vector2> pathNodes = {Vector2(0,0), Vector2(1,2), Vector2(2,2), Vector2(3,0), Vector2(5,0), Vector2(6,3)};

    /// Run program
    CurvatureOptimizer curvatureOptimizer;
    curvatureOptimizer.setControlPoints(controlPoints);
    curvatureOptimizer.setPathNodes(pathNodes);
    float curvatureBefore = curvatureOptimizer.calculateMaximumCurvature();
    curvatureOptimizer.optimizeControlPoints();
    float curvatureAfter = curvatureOptimizer.calculateMaximumCurvature();

    /// Test if we did okay
    std::cout << "max curvature before: " << curvatureBefore << std::endl;
    std::cout << "max curvature after: " << curvatureAfter << std::endl;
    if (curvatureAfter < curvatureBefore) {
        ASSERT_TRUE(true);
    } else {
        ASSERT_FALSE(true);
    }
}
}
}