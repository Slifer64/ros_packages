#ifndef SIM_ROBOT_ARM_H
#define SIM_ROBOT_ARM_H

#include <robot_sim/robot_chain.h>

namespace as64_
{

namespace robot_sim_
{

class RobotArm : public RobotChain
{
public:
  RobotArm() {}
  RobotArm(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity, const std::string &wrench_topic="") :
    RobotChain(urdf_model, base_link, tool_link, ctrl_cycle, check_limits, check_singularity, wrench_topic)
    {}
};

}; // namespace robot_sim_

}; // namespace as64_

#endif // SIM_ROBOT_CHAIN_H
