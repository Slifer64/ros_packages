#include <iostream>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <armadillo>
#include <robot_sim/lwr4p_robot.h>

using namespace as64_;

double err_thres = 1e-2;

std::shared_ptr<robot_sim_::LWR4pRobot> lwr4p_robot;

void PRINT_INFO_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[34m" << "[RobotSim INFO]: " << msg << "\033[0m\n";
}

void PRINT_ERR_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[31m" << "[RobotSim ERROR]: " << msg << "\033[0m\n";
}

void jointsTrajectory(arma::vec qT, double total_time)
{
  // ======================================================
  // ===========   Set Joints trajectory  =================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Joints Trajectory...");
  bool reached_target = lwr4p_robot->setJointsTrajectory(qT, total_time);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!\n");
  if (!lwr4p_robot->isOk())
  {
    PRINT_ERR_MSG("==> " + lwr4p_robot->getErrMsg());
    lwr4p_robot->enable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void jointPositionControl(arma::vec qT, double total_time)
{
  // ======================================================
  // ===========   Joint Position Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_POS_CONTROL...");
  lwr4p_robot->setMode(robot_sim_::JOINT_POS_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint position control...");
  lwr4p_robot->update();
  double Ts = lwr4p_robot->getCtrlCycle();
  arma::vec q = lwr4p_robot->getJointsPosition();
  arma::vec q0 = q;
  double t = 0;
  while (arma::norm(qT-q)>err_thres && lwr4p_robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = robot_sim_::get5thOrder(t, q0, qT, total_time)[0];
    lwr4p_robot->setJointsPosition(q_ref);
    lwr4p_robot->update();
    q = lwr4p_robot->getJointsPosition();
  }
  if (!lwr4p_robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + lwr4p_robot->getErrMsg());
    lwr4p_robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void jointVelocityControl(arma::vec qT, double total_time)
{
  // ======================================================
  // ===========   Joint Velocity Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_VEL_CONTROL...");
  lwr4p_robot->setMode(robot_sim_::JOINT_VEL_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint velocity control...");
  lwr4p_robot->update();
  double Ts = lwr4p_robot->getCtrlCycle();
  arma::vec q = lwr4p_robot->getJointsPosition();
  arma::vec q0 = q;
  double k_click = 0.1;
  double t = 0;
  while (arma::norm(qT-q)>err_thres && lwr4p_robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = robot_sim_::get5thOrder(t, q0, qT, total_time)[0];
    arma::vec dq_ref = robot_sim_::get5thOrder(t, q0, qT, total_time)[1];
    lwr4p_robot->setJointsVelocity(dq_ref + k_click*(q_ref-q));
    lwr4p_robot->update();
    q = lwr4p_robot->getJointsPosition();
  }
  if (!lwr4p_robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + lwr4p_robot->getErrMsg());
    lwr4p_robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void cartesianVelocityControl(arma::vec qT, double total_time)
{
  // ==========================================================
  // ===========   Cartesian Velocity Control  ================
  // ==========================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to CART_VEL_CONTROL...");
  lwr4p_robot->setMode(robot_sim_::CART_VEL_CONTROL);
  PRINT_INFO_MSG("==> Moving with Cartesian velocity control...");
  lwr4p_robot->update();
  arma::vec q = lwr4p_robot->getJointsPosition();
  arma::vec q0 = q;
  arma::mat p0 = lwr4p_robot->getTaskPose(q0);
  arma::mat pT = lwr4p_robot->getTaskPose(qT);
  arma::mat p = lwr4p_robot->getTaskPose();
  double t = 0;
  double Ts = lwr4p_robot->getCtrlCycle();
  // Notice that if we were to check the joint position instead of the Cartesian we may never
  // reach qT, because due to pinv(J) we may arrive at pT with a different joint configuration.
  while (arma::norm(arma::vectorise(pT)-arma::vectorise(p))>err_thres && lwr4p_robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = robot_sim_::get5thOrder(t, q0, qT, total_time)[0];
    // arma::vec dq_ref = robot_sim_::get5thOrder(t, q0, qT, total_time)[1];
    arma::vec v_click(6);
    arma::mat p_ref = lwr4p_robot->getTaskPose(q_ref);
    // position error
    v_click.subvec(0,2) = p_ref.submat(0,3,2,3) - p.submat(0,3,2,3);
    // orientation error (difference of quaternions vector parts)
    v_click.subvec(3,5) = robot_sim_::rotm2quat(p_ref.submat(0,0,2,2)).subvec(1,3) - robot_sim_::rotm2quat(p.submat(0,0,2,2)).subvec(1,3);
    // v_click *= 4.0;
    // arma::vec V = lwr4p_robot->getJacobian()*dq_ref + v_click;
    arma::vec V = v_click/Ts;
    lwr4p_robot->setTaskVelocity(V);
    lwr4p_robot->update();
    p = lwr4p_robot->getTaskPose();
  }
  if (!lwr4p_robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + lwr4p_robot->getErrMsg());
    lwr4p_robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void taskTrajectory(arma::vec qT, double total_time)
{
  // ======================================================
  // ===========   Set Task trajectory  =================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Task Trajectory...");
  arma::mat target_pose = lwr4p_robot->getTaskPose(qT);
  bool reached_target = lwr4p_robot->setTaskTrajectory(target_pose, total_time);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!");
  if (!lwr4p_robot->isOk())
  {
    PRINT_ERR_MSG("==> " + lwr4p_robot->getErrMsg());
    lwr4p_robot->enable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void freedrive()
{
  // =========================================
  // ===========   FREEDIRVE  ================
  // =========================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to FREEDRIVE...");
  lwr4p_robot->setMode(robot_sim_::FREEDRIVE);
  PRINT_INFO_MSG("==> The robot is in FREEDIRVE. Move it wherever you want. Press ctrl+C to exit...");

  while (ros::ok() && lwr4p_robot->isOk())
  {
    lwr4p_robot->update();
  }

  if (!lwr4p_robot->isOk())
  {
    PRINT_INFO_MSG("[ERROR]: " + lwr4p_robot->getErrMsg());
    lwr4p_robot->enable();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwr4p_robot_sim");

  arma::vec q1;
  arma::vec q2;
  double time_duration; // sec

  ros::NodeHandle nh("~");
  std::vector<double> q_vec;

  if (nh.getParam("q1",q_vec)) q1 = q_vec;
  else  q1 = {-0.5, 0.6, 0.3, -1.5, 0, -0.2, 1.1};

  if (nh.getParam("q2",q_vec)) q2 = q_vec;
  else  q2 = {0.8, 0.9, -0.1, -1.3, -0.1, -1.2, -1.3};

  if (!nh.getParam("time_duration",time_duration)) time_duration = 5.0;

  std::cout << "=======================================\n";
  std::cout << "q1 = " << q1.t() << "\n";
  std::cout << "q2 = " << q2.t() << "\n";
  std::cout << "time_duration = " << time_duration << "\n";
  std::cout << "=======================================\n";

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // initialize robot
  lwr4p_robot.reset(new robot_sim_::LWR4pRobot());

  // ===========   Set Joints trajectory  =================
  jointsTrajectory(q1, time_duration);

  // ===========   Joint Position Control  ================
  jointPositionControl(q2, time_duration);

  // ===========   Joint Velocity Control  ================
  jointVelocityControl(q1, time_duration);

  // ===========   Cartesian Velocity Control  ================
  cartesianVelocityControl(q2, time_duration);

  // ===========   Set Tasktrajectory  =================
  taskTrajectory(q1, time_duration);

  // ===========   FREEDIRVE  ================
  freedrive();

  return 0;
}
