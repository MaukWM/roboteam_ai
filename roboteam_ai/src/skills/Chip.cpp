//
// Created by mrlukasbos on 23-10-18.
//

#include "Chip.h"

namespace rtt {
namespace ai {

void Chip::sendKickCommand(double kickVel) {
  roboteam_msgs::RobotCommand command;
  command.id = robot.id;
  // TODO check if we can avoid the casting to unsigned char without warnings
  command.chipper = (unsigned char) true;
  command.chipper_forced = (unsigned char) true;
  command.kicker_vel = (float) kickVel;
}

} // ai
} // rtt