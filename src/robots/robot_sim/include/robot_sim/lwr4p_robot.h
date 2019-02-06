#ifndef SIM_LWR4P_ROBOT_H
#define SIM_LWR4P_ROBOT_H

#include <robot_sim/robot_arm.h>

namespace as64_
{

namespace robot_sim_
{

class LWR4pRobot: public RobotArm
{
public:
  LWR4pRobot();
  LWR4pRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity, const std::string &wrench_topic="") :
    RobotArm(urdf_model, base_link, tool_link, ctrl_cycle, check_limits, check_singularity, wrench_topic)
    {}
};

}; // namespace robot_sim_

}; // namespace as64_

#endif // SIM_LWR4P_ROBOT_H
