#ifndef SIM_BH282_ROBOT_H
#define SIM_BH282_ROBOT_H

#include <bhand/robot_hand.h>

namespace as64_
{

namespace bhand_
{

class Bh282SimRobot : public RobotHand
{
public:
  Bh282SimRobot();
  Bh282SimRobot(urdf::Model &urdf_model, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity, const std::string &pub_state_topic, const std::vector<std::string> &wrench_topic):
    RobotHand(urdf_model, base_link, tool_link, ctrl_cycle, check_limits, check_singularity, pub_state_topic, wrench_topic) {}

  int getNumJoints() const { return 4; }
};

}; // namespace bhand_

}; // namespace as64_

#endif // SIM_BH282_ROBOT_H
