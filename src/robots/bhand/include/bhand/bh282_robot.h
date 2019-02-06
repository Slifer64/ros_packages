#ifndef BH282_ROBOT_H
#define BH282_ROBOT_H

#include <bhand/robot_hand.h>
#include <bhand/bhand_API/BhandHWInterface.h>

namespace as64_
{

namespace bhand_
{

class Bh282Robot : public RobotHand
{
public:
  Bh282Robot();
  Bh282Robot(urdf::Model &urdf_model, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity, const std::string &pub_state_topic, const std::vector<std::string> &wrench_topic):
    RobotHand(urdf_model, base_link, tool_link, ctrl_cycle, check_limits, check_singularity, pub_state_topic, wrench_topic) {}
  ~Bh282Robot();

  void update();
  void setMode(const bhand_::Mode &m);

  void setJointPosition(double pos, bhand_::JointName jn);
  void setJointVelocity(double vel, bhand_::JointName jn);
  double getJointTorque(bhand_::JointName jn);

private:
  BhandHWInterface hw_i;

  arma::vec getActualPosition();
  void checkJointPosDeviationError();

  double k_click;

  // freedrive model
  arma::vec q_a, dq_a, ddq_a;
  double p_a; // admittance triple pole
  double t_s; // torque scaling factor

  void initFreedrive();
  void updateFreedrive();
};

}; // namespace bhand_

}; // namespace as64_

#endif // BH282_ROBOT_H