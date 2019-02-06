#ifndef LWR4P_ROBOT_ARM_H
#define LWR4P_ROBOT_ARM_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <memory>
#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
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
#include <lwr4p/utils.h>

namespace as64_
{

namespace lwr4p_
{

class RobotArm
{
public:
  RobotArm();
  RobotArm(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link, double ctrl_cycle);
  ~RobotArm();

  void setJointLimitCheck(bool check);
  void setSingularityCheck(bool check);
  void setSingularityThreshold(double thres);
  void readWrenchFromTopic(bool set, const std::string &topic="");

  virtual bool isOk() const;
  virtual void enable();
  std::string getErrMsg() const;
  lwr4p_::Mode getMode() const;
  double getCtrlCycle() const;
  int getNumJoints() const;
  bool setJointsTrajectory(const arma::vec &j_targ, double duration);
  bool setTaskTrajectory(const arma::mat &target_pose, double duration);

  virtual void update() = 0;
  virtual void setMode(const lwr4p_::Mode &m) = 0;
  virtual void setJointsPosition(const arma::vec &j_pos) = 0;
  virtual void setJointsVelocity(const arma::vec &j_vel) = 0;
  virtual void setTaskVelocity(const arma::vec &task_vel) = 0;
  virtual void setJointsTorque(const arma::vec &j_torques) = 0;
  virtual void setTaskPose(const arma::mat &task_pose) = 0;
  virtual void setWrench(const arma::vec &wrench) = 0;

  virtual void setCartStiffness(const arma::vec &cart_stiff);
  virtual void setCartDamping(const arma::vec &cart_damp);

  virtual arma::vec getJointsPosition() const;
  virtual arma::vec getJointsVelocity() const;
  virtual arma::mat getTaskPose() const;
  virtual arma::vec getTaskPosition() const;
  virtual arma::vec getTaskOrientation() const;
  virtual arma::mat getJacobian() const;
  virtual arma::mat getEEJacobian() const;
  virtual arma::vec getJointsTorque() const;
  virtual arma::vec getExternalWrench() const;

  // return using the urdf model and kdl forward-inverse kinematics solvers
  arma::vec getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution=NULL) const;
  arma::mat getTaskPose(const arma::vec &j_pos) const;
  arma::mat getJacobian(const arma::vec j_pos) const;

  void addJointState(sensor_msgs::JointState &joint_state_msg);

protected:

  void setJointsPositionHelper(const arma::vec &j_pos);
  void setJointsVelocityHelper(const arma::vec &j_vel);
  void setTaskVelocityHelper(const arma::vec &task_vel);

  void init();

  Mode mode;
  std::map<lwr4p_::Mode, std::string> mode_name;

  int N_JOINTS;

  double SINGULARITY_THRES;

  std::mutex robot_state_mtx;

  urdf::Model urdf_model;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;//Inverse velocity solver
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;

  ros::NodeHandle node;
  // ros::Publisher jState_pub; ///< joint state publisher
  ros::Subscriber jState_sub; ///< joint state subscriber

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;

  std::string base_link_name;
  std::string tool_link_name;

  double ctrl_cycle;
  arma::vec prev_joint_pos;
  arma::vec joint_pos;
  arma::vec Fext;

  arma::vec cart_stiff;
  arma::vec cart_damp;

  bool check_limits;
  bool check_singularity;

  bool read_wrench_from_topic;
  std::string wrench_topic;
  ros::Subscriber wrench_sub;

  std::string err_msg;
  bool checkJointPosLimits(const arma::vec &j_pos);
  bool checkJointVelLimits(const arma::vec &dj_pos);
  bool checkSingularity();

  virtual void stopController() = 0;
  void protectiveStop();

  std::string getModeName(Mode mode) const;

  void readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr);
};

}; // namespace lwr4p_

}; // namespace as64_

#endif // LWR4P_ROBOT_ARM_H