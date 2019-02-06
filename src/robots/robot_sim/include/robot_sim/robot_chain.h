#ifndef SIM_ROBOT_CHAIN_H
#define SIM_ROBOT_CHAIN_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

#include <armadillo>
#include <robot_sim/utils.h>

namespace as64_
{

namespace robot_sim_
{

enum Mode
{
  IDLE,
  FREEDRIVE,
  JOINT_POS_CONTROL,
  JOINT_VEL_CONTROL,
  CART_VEL_CONTROL,
  PROTECTIVE_STOP
};

class RobotChain
{
public:
  RobotChain();
  RobotChain(urdf::Model &urdf_model,
    const std::string &base_link, const std::string &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity,
    const std::string &wrench_topic="");
  ~RobotChain();

  bool isOk() const;
  void enable();
  std::string getErrMsg() const;

  void update();

  void setMode(const robot_sim_::Mode &m);
  robot_sim_::Mode getMode() const;

  double getCtrlCycle() const;
  int getNumJoints() const;

  void setJointsPosition(const arma::vec &j_pos);
  void setJointsVelocity(const arma::vec &j_vel);
  void setTaskVelocity(const arma::vec &task_vel);
  bool setJointsTrajectory(const arma::vec &j_targ, double duration);
  bool setTaskTrajectory(const arma::mat &target_pose, double duration);

  arma::vec getJointsPosition() const;
  arma::vec getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution=NULL) const;
  arma::vec getJointsVelocity() const;
  arma::mat getTaskPose() const;
  arma::mat getTaskPose(const arma::vec &j_pos) const;
  arma::vec getTaskPosition() const;
  arma::vec getTaskOrientation() const;
  arma::mat getJacobian() const;
  arma::mat getJacobian(const arma::vec j_pos) const;
  arma::mat getEEJacobian() const;
  arma::vec getJointTorques() const;
  arma::vec getExternalForce() const;

  void printRobotInfo() const;

  void updateState();
  void publishState();

  std::vector<std::string> getJointNames() const { return joint_names; }

  void addJointState(sensor_msgs::JointState &joint_state_msg);

protected:

  void init();

  std::vector<std::string> mode_name;

  Mode mode;

  int N_JOINTS;

  double SINGULARITY_THRES;

  std::mutex robot_state_mtx;

  urdf::Model urdf_model;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;//Inverse velocity solver
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;

  std::string pub_topic;
  ros::NodeHandle node;
  ros::Publisher jState_pub; ///< joint state publisher
  ros::Subscriber jState_sub; ///< joint state subscriber

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;

  std::string base_link_name;
  std::string tool_link_name;

  double ctrl_cycle;
  unsigned long update_time;
  robot_sim_::Timer timer;
  arma::vec joint_pos;
  arma::vec joint_prev_pos;
  arma::vec joint_vel;
  arma::vec joint_torques;
  arma::mat pose;
  arma::mat Jrobot;
  arma::mat Jee;
  arma::vec Fext;

  bool check_limits;
  bool  check_singularity;

  bool read_wrench_from_topic;
  std::string wrench_topic;
  ros::Subscriber wrench_sub;

  std::string err_msg;
  bool checkJointPosLimits(const arma::vec &j_pos);
  bool checkJointVelLimits(const arma::vec &dj_pos);
  bool checkSingularity();

  void stop();
  void protectiveStop();

  std::string getModeName() const;

  void setJointsPositionHelper(const arma::vec &j_pos);
  void setJointsVelocityHelper(const arma::vec &j_vel);
  void setTaskVelocityHelper(const arma::vec &task_vel);

  void jStateSubCallback(const sensor_msgs::JointState::ConstPtr& jState);

  void readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr);
};

}; // namespace robot_sim_

}; // namespace as64_

#endif // SIM_ROBOT_CHAIN_H
