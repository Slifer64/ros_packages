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
#include <bhand/utils.h>

namespace as64_
{

namespace bhand_
{

class RobotChain
{
public:
  RobotChain(urdf::Model &urdf_model,
    const std::string &base_link, const std::string &tool_link,
    double ctrl_cycle, bool check_limits, bool check_singularity,
    const std::string &pub_state_topic, const std::string &wrench_topic="");
  ~RobotChain();

  bool isOk() const;
  void enable();
  std::string getErrMsg() const;

  void setMode(const bhand_::Mode &m);
  bhand_::Mode getMode() const;

  double getCtrlCycle() const;
  int getNumJoints() const;

  void setJointPosition(const arma::vec &j_pos);
  void setJointVelocity(const arma::vec &j_vel);
  void setTaskVelocity(const arma::vec &task_vel);

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

  void updateState();

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

  std::string pub_state_topic;
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

  void setJointPositionHelper(const arma::vec &j_pos);
  void setJointVelocityHelper(const arma::vec &j_vel);
  void setTaskVelocityHelper(const arma::vec &task_vel);

  void jStateSubCallback(const sensor_msgs::JointState::ConstPtr& jState);

  void readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr);
};

}; // namespace bhand_

}; // namespace as64_

#endif // SIM_ROBOT_CHAIN_H
