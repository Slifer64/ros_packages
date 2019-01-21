#ifndef SIMULATED_ROBOT_HAND_H
#define SIMULATED_ROBOT_HAND_H

#include <robot_sim/robot_chain.h>

namespace as64_
{

namespace robot_sim_
{

class RobotHand
{
public:
  RobotHand();
  RobotHand(urdf::Model &urdf_model, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity, const std::vector<std::string> &wrench_topic);
  ~RobotHand();

  bool isOk() const;
  void enable();
  std::string getErrMsg() const;

  virtual void update();

  void setMode(const robot_sim_::Mode &m);
  robot_sim_::Mode getMode() const { return mode; }

  double getCtrlCycle() const { return ctrl_cycle; }
  int getNumJoints(int fing_ind) const { return fingers[fing_ind]->getNumJoints(); }
  int getNumFingers() const { return N_fingers; }

  void setJointsPosition(const arma::vec &j_pos, int fing_ind) { fingers[fing_ind]->setJointsPosition(j_pos); };
  void setJointsVelocity(const arma::vec &j_vel, int fing_ind) { fingers[fing_ind]->setJointsVelocity(j_vel); };
  void setTaskVelocity(const arma::vec &task_vel, int fing_ind) { fingers[fing_ind]->setTaskVelocity(task_vel); };
  void setJointsTrajectory(const arma::vec &j_targ, double duration, int fing_ind) { fingers[fing_ind]->setJointsTrajectory(j_targ,duration); };

  arma::vec getJointsPosition(int fing_ind) const { return fingers[fing_ind]->getJointsPosition(); };
  arma::vec getJointsVelocity(int fing_ind) const { return fingers[fing_ind]->getJointsVelocity(); };
  arma::mat getTaskPose(int fing_ind) const { return fingers[fing_ind]->getTaskPose(); };
  arma::vec getTaskPosition(int fing_ind) const { return fingers[fing_ind]->getTaskPosition(); };
  arma::vec getTaskOrientation(int fing_ind) const { return fingers[fing_ind]->getTaskOrientation(); };
  arma::mat getJacobian(int fing_ind) const { return fingers[fing_ind]->getJacobian(); };
  arma::mat getEEJacobian(int fing_ind) const { return fingers[fing_ind]->getEEJacobian(); };
  arma::vec getJointTorques(int fing_ind) const { return fingers[fing_ind]->getJointTorques(); };
  arma::vec getExternalForce(int fing_ind) const { return fingers[fing_ind]->getExternalForce(); };

protected:

  void init();

  unsigned long update_time;
  robot_sim_::Timer timer;

  robot_sim_::Mode mode;

  double ctrl_cycle;
  bool check_limits;
  bool check_singularity;

  int N_fingers;
  std::vector<std::string> base_link;
  std::vector<std::string> tool_link;
  std::vector<std::string> wrench_topic;
  std::vector<std::shared_ptr<RobotChain>> fingers;

  std::string pub_topic;
  ros::NodeHandle node;
  ros::Publisher jState_pub; ///< joint state publisher

  urdf::Model urdf_model;
};

}; // namespace robot_sim_

}; // namespace as64_

#endif // SIMULATED_ROBOT_HAND_H
