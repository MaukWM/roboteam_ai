//
// Created by baris on 6-2-19.
//

#include "HaltTactic.h"


bt::HaltTactic::HaltTactic(std::string name, Blackboard::Ptr blackboard) {

}
void bt::HaltTactic::initialize() {
    Tactic::initialize();
}
bt::Node::Status bt::HaltTactic::update() {
    return Tactic::update();
}
void bt::HaltTactic::claimRobots() {

}
