#ifndef BH282_SIM_ROBOT_H
#define BH282_SIM_ROBOT_H

#include <bhand/robot_hand.h>

namespace as64_
{

namespace bhand_
{

class Bh282SimRobot : public RobotHand
{
public:
  Bh282SimRobot();
  Bh282SimRobot(urdf::Model &urdf_model, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  Bh282SimRobot(const std::string &robot_desc_param, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);


  int getNumJoints() const { return 4; }
};

}; // namespace bhand_

}; // namespace as64_

#endif // BH282_SIM_ROBOT_H
