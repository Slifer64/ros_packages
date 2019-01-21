#ifndef SIMULATED_ROBOT_HAND_H
#define SIMULATED_ROBOT_HAND_H

#include <bhand/robot_chain.h>

namespace as64_
{

namespace bhand_
{

class RobotHand
{
public:
  RobotHand() {}
  RobotHand(urdf::Model &urdf_model, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity, const std::string &pub_state_topic, const std::vector<std::string> &wrench_topic);
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

  virtual void setJointPosition(double pos, bhand_::JointName jn);
  void setJointPosition(arma::vec j_pos, const std::vector<bhand_::JointName> &jn={PACK,FING1,FING2,FING3});
  virtual void setJointVelocity(double vel, bhand_::JointName jn);
  void setJointVelocity(arma::vec j_vel, const std::vector<bhand_::JointName> &jn={PACK,FING1,FING2,FING3});
  virtual bool setJointTrajectory(arma::vec j_target, double duration, const std::vector<bhand_::JointName> &jn);

  virtual double getJointPosition(bhand_::JointName jn) const;
  arma::vec getJointPosition(const std::vector<bhand_::JointName> &jn={PACK,FING1,FING2,FING3}) const;
  virtual double getJointVelocity(bhand_::JointName jn) const;
  arma::vec getJointVelocity(const std::vector<bhand_::JointName> &jn={PACK,FING1,FING2,FING3}) const;

  virtual void setTaskVelocity(const arma::vec &task_vel, bhand_::ChainName chain_ind) { fingers[(int)chain_ind]->setTaskVelocity(task_vel); };
  virtual arma::mat getTaskPose(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getTaskPose(); };
  virtual arma::vec getTaskPosition(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getTaskPosition(); };
  virtual arma::vec getTaskOrientation(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getTaskOrientation(); };
  virtual arma::mat getJacobian(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getJacobian(); };
  virtual arma::mat getEEJacobian(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getEEJacobian(); };
  virtual arma::vec getJointTorques(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getJointTorques(); };
  virtual arma::vec getExternalForce(bhand_::ChainName chain_ind) const { return fingers[(int)chain_ind]->getExternalForce(); };

protected:
  void getChainFingerInd(bhand_::JointName jn, int &chain_ind, int &joint_ind) const;
  void init();
  void waitNextCycle();
  void updateState();
  void publishState();

  unsigned long update_time;
  bhand_::Timer timer;

  bhand_::Mode mode;

  double ctrl_cycle;
  bool check_limits;
  bool check_singularity;

  int N_fingers;
  std::vector<std::string> base_link;
  std::vector<std::string> tool_link;
  std::vector<std::string> wrench_topic;
  std::vector<std::shared_ptr<RobotChain>> fingers;

  std::string pub_state_topic;
  ros::NodeHandle node;
  ros::Publisher jState_pub; ///< joint state publisher

  urdf::Model urdf_model;
};

}; // namespace bhand_

}; // namespace as64_

#endif // SIMULATED_ROBOT_HAND_H
