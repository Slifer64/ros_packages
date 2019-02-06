#ifndef SIM_BH282_ROBOT_H
#define SIM_BH282_ROBOT_H

#include <robot_sim/robot_hand.h>

namespace as64_
{

namespace robot_sim_
{

class Bh282Robot : public RobotHand
{
public:
  enum JointName
  {
    PACK = 0,
    FING1 = 1,
    FING2 = 2,
    FING3 = 3
  };

  Bh282Robot();
  Bh282Robot(urdf::Model &urdf_model, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity, const std::vector<std::string> &wrench_topic):
    RobotHand(urdf_model, base_link, tool_link, ctrl_cycle, check_limits, check_singularity, wrench_topic) {}

  // bool isOk() const;
  // void enable();
  // std::string getErrMsg() const;

  // virtual void update();

  // void setMode(const robot_sim_::Mode &m);
  // robot_sim_::Mode getMode() const { return mode; }

  // double getCtrlCycle() const { return ctrl_cycle; }
  // int getNumFingers() const { return N_fingers; }
  int getNumJoints() const { return 4; }

  void setJointPos(double pos, Bh282Robot::JointName jn);
  void setJointVel(double vel, Bh282Robot::JointName jn);

  bool setJointsTraj(arma::vec j_target, double duration, const std::vector<Bh282Robot::JointName> &jn);

  double getJointPos(Bh282Robot::JointName jn) const;
  double getJointVel(Bh282Robot::JointName jn) const;

  // arma::mat getTaskPose(int fing_ind) const;
  // arma::vec getTaskPosition(int fing_ind) const;
  // arma::vec getTaskOrientation(int fing_ind) const;
  // arma::mat getJacobian(int fing_ind) const;
  // arma::mat getEEJacobian(int fing_ind) const;
  // arma::vec getJointTorques(int fing_ind) const;
  // arma::vec getExternalForce(int fing_ind) const;
private:
  void getChainFingerInd(Bh282Robot::JointName jn, int &chain_ind, int &joint_ind) const;
};

}; // namespace robot_sim_

}; // namespace as64_

#endif // SIM_BH282_ROBOT_H
