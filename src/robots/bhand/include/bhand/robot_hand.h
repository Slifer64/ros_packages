#ifndef BHAND_ROBOT_HAND_H
#define BHAND_ROBOT_HAND_H

#include <bhand/robot_chain.h>

namespace as64_
{

namespace bhand_
{

class RobotHand
{
public:
  RobotHand() {}
  RobotHand(urdf::Model &urdf_model, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  RobotHand(const std::string &robot_desc_param, const std::string &base_link, const std::vector<std::string> &tool_link, double ctrl_cycle);
  ~RobotHand();

  virtual bool isOk() const;
  virtual void enable();
  virtual void update();

  int getNumJoints() const { return 4; }
  std::string getErrMsg() const;

  virtual void setMode(const bhand_::Mode &m);
  bhand_::Mode getMode() const { return mode; }

  double getCtrlCycle() const { return ctrl_cycle; }

  int getNumJoints(int fing_ind) const { return fingers[fing_ind]->getNumJoints(); }
  int getNumFingers() const { return N_fingers; }

  virtual void setJointsPosition(double pos, bhand_::JointName jn);
  void setJointsPosition(arma::vec j_pos, const std::vector<bhand_::JointName> &jn={SPREAD,FING1,FING2,FING3});
  virtual void setJointsVelocity(double vel, bhand_::JointName jn);
  void setJointsVelocity(arma::vec j_vel, const std::vector<bhand_::JointName> &jn={SPREAD,FING1,FING2,FING3});
  virtual bool setJointsTrajectory(arma::vec j_target, double duration, const std::vector<bhand_::JointName> &jn);

  virtual double getJointPosition(bhand_::JointName jn) const;
  arma::vec getJointPosition(const std::vector<bhand_::JointName> &jn={SPREAD,FING1,FING2,FING3}) const;
  virtual double getJointVelocity(bhand_::JointName jn) const;
  arma::vec getJointVelocity(const std::vector<bhand_::JointName> &jn={SPREAD,FING1,FING2,FING3}) const;

  virtual void setTaskVelocity(const arma::vec &task_vel, bhand_::ChainName chain_ind) { fingers[(int)chain_ind]->setTaskVelocity(task_vel); };
  virtual arma::mat getTaskPose(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getTaskPose(); };
  virtual arma::vec getTaskPosition(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getTaskPosition(); };
  virtual arma::vec getTaskOrientation(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getTaskOrientation(); };
  virtual arma::mat getJacobian(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getJacobian(); };
  virtual arma::mat getEEJacobian(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getEEJacobian(); };
  virtual arma::vec getJointTorques(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getJointTorques(); };
  virtual arma::vec getExternalForce(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getExternalForce(); };

  void addJointState(sensor_msgs::JointState &joint_state_msg);

protected:

  void setJointsPositionHelper(const arma::vec &j_pos);
  void setJointsVelocityHelper(const arma::vec &j_vel);
  void setTaskVelocityHelper(const arma::vec &task_vel);

  void getChainFingerInd(bhand_::JointName jn, int &chain_ind, int &joint_ind) const;
  void init();

  unsigned long update_time;
  bhand_::Timer timer;

  bhand_::Mode mode;

  double ctrl_cycle;
  bool check_limits;
  bool check_singularity;

  int N_fingers;
  std::string base_link;
  std::vector<std::string> tool_link;
  std::vector<std::string> wrench_topic;
  std::vector<std::shared_ptr<KinematicChain>> fingers;

  urdf::Model urdf_model;
};

}; // namespace bhand_

}; // namespace as64_

#endif // BHAND_ROBOT_HAND_H
