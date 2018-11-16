#include "ros/ros.h"
#include "DangerFinder/DangerFinder.h"
#include "io/IOManager.h"
#include "treeinterp/TreeInterpreter.h"
#include "utilities/Referee.hpp"
#include "utilities/StrategyManager.h"
#include "treeinterp/BTFactory.h"

namespace df = rtt::ai::dangerfinder;
namespace io = rtt::ai::io;
namespace ai = rtt::ai;

roboteam_msgs::World worldMsg;

using Status = bt::Node::Status;

int main(int argc, char* argv[]) {
    // Init ROS node
    ros::init(argc, argv, "StrategyNode");

    // init IOManager and subscribe to all topics immediately
    io::IOManager IOManager(true);
    roboteam_msgs::World worldMsg;
    roboteam_msgs::GeometryData geometryMsg;
    roboteam_msgs::RefereeData refereeMsg;

    bt::BehaviorTree::Ptr strategy;

    // start looping
    // set the framerate to 50 Hz
    ros::Rate rate(50);

    auto factory = BTFactory::getFactory();

    factory.init();

    std::string currentTree = "victoryDanceStrategy";

    while (ros::ok()) {
        ros::spinOnce();

        // make ROS worldstate and geometry data globally accessible
        worldMsg = IOManager.getWorldState();
        geometryMsg = IOManager.getGeometryData();
        refereeMsg = IOManager.getRefereeData();
        ai::World::set_world(worldMsg);
        ai::Field::set_field(geometryMsg.field);
        ai::Referee::setRefereeData(refereeMsg);

        if (!ai::World::didReceiveFirstWorld) continue;


        // for refereedata:
        // ai::StrategyManager strategyManager;
        // std::string strategyName = strategyManager.getCurrentStrategyName();
        // strategy = factory.getTree(strategyName);

        strategy = factory.getTree(currentTree);

        Status status = strategy->Tick();

        if (status != Status::Running) {
            std::string statusStr = bt::statusToString(status);
            // return failure, success or invalid
            ROS_DEBUG_STREAM_NAMED("Roboteam_ai", "Strategy result: " << statusStr.c_str() << "Shutting down...\n");
            if (status == Status::Success) {
                std::cerr << "===================================== TREE CHANGE =====================================================" << std::endl;
                currentTree = "ParallelSequenceStrategy"; // TODO give new tree name
                continue;
            } else if (status == Status::Failure) {
                std::cerr << "fail...." << std::endl;
            }
        }
        rate.sleep();
    }

    // Terminate if needed
    if (strategy->getStatus() == Status::Running) {
        strategy->Terminate(Status::Running);
    }

    return 0;
}
