#include <iostream>
#include <cstdlib>
#include <memory>

#include <ros/ros.h>
#include <bhand/bh282_robot.h>
#include <bhand/bh282_sim_robot.h>

using namespace as64_;

double err_thres = 1e-2;

std::shared_ptr<bhand_::RobotHand> bh282_robot;

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
  // ===========   Set Joints trajectory  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Joints Trajectory...");
  std::vector<bhand_::JointName> jn = {bhand_::PACK, bhand_::FING1, bhand_::FING2, bhand_::FING3};
  bool reached_target = bh282_robot->setJointTrajectory(qT, total_time, jn);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!\n");
  if (!bh282_robot->isOk())
  {
    PRINT_ERR_MSG("==> " + bh282_robot->getErrMsg());
    bh282_robot->enable();
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
  bh282_robot->setMode(bhand_::JOINT_POS_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint position control...");
  bh282_robot->update();
  double Ts = bh282_robot->getCtrlCycle();
  int N_joints = bh282_robot->getNumJoints();
  arma::vec q(N_joints); // {PACK, FING1, FING2, FING3}
  for (int i=0;i<N_joints;i++) q(i) = bh282_robot->getJointPosition((bhand_::JointName)i);
  arma::vec q0 = q;
  double t = 0;
  while (arma::norm(qT-q)>err_thres && bh282_robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = bhand_::get5thOrder(t, q0, qT, total_time)[0];
    //for (int i=0;i<N_joints && bh282_robot->isOk(); i++) bh282_robot->setJointPosition(q_ref(i), (bhand_::JointName)i);
    bh282_robot->setJointPosition(q_ref, {bhand_::PACK, bhand_::FING1, bhand_::FING2, bhand_::FING3});
    bh282_robot->update();
    for (int i=0;i<N_joints;i++) q(i) = bh282_robot->getJointPosition((bhand_::JointName)i);
  }
  if (!bh282_robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + bh282_robot->getErrMsg());
    bh282_robot->enable();
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
  bh282_robot->setMode(bhand_::JOINT_VEL_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint velocity control...");
  bh282_robot->update();
  double Ts = bh282_robot->getCtrlCycle();
  int N_joints = bh282_robot->getNumJoints();
  arma::vec q(N_joints); // {PACK, FING1, FING2, FING3}
  for (int i=0;i<N_joints;i++) q(i) = bh282_robot->getJointPosition((bhand_::JointName)i);
  arma::vec q0 = q;
  double k_click = 0.0;
  double t = 0;
  while (arma::norm(qT-q)>err_thres && bh282_robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = bhand_::get5thOrder(t, q0, qT, total_time)[0];
    arma::vec dq_ref = bhand_::get5thOrder(t, q0, qT, total_time)[1];
    arma::vec dq_cmd = dq_ref + k_click*(q_ref-q);
    for (int i=0;i<N_joints && bh282_robot->isOk(); i++) bh282_robot->setJointVelocity(dq_cmd(i), (bhand_::JointName)i);
    // bh282_robot->setJointVelocity(dq_cmd, {bhand_::PACK, bhand_::FING1, bhand_::FING2, bhand_::FING3});
    bh282_robot->update();
    for (int i=0;i<N_joints;i++) q(i) = bh282_robot->getJointPosition((bhand_::JointName)i);
  }
  if (!bh282_robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + bh282_robot->getErrMsg());
    bh282_robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void freedrive()
{
  // =========================================
  // ===========   FREEDIRVE  ================
  // =========================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to FREEDRIVE...");
  bh282_robot->setMode(bhand_::FREEDRIVE);
  PRINT_INFO_MSG("==> The robot is in FREEDIRVE. Move it wherever you want. Press ctrl+C to exit...");

  while (ros::ok() && bh282_robot->isOk())
  {
    bh282_robot->update();
  }

  if (!bh282_robot->isOk())
  {
    PRINT_INFO_MSG("[ERROR]: " + bh282_robot->getErrMsg());
    bh282_robot->enable();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bh282_bhand");

  arma::vec q1;
  arma::vec q2;
  double time_duration; // sec
  bool use_sim;

  ros::NodeHandle nh("~");
  std::vector<double> q_vec;

  if (nh.getParam("q1",q_vec)) q1 = q_vec;
  else  q1 = {0.0, 0.0, 0.0, 0.0};

  if (nh.getParam("q2",q_vec)) q2 = q_vec;
  else  q2 = {0.5, 0.5, 0.5, 0.5};

  if (!nh.getParam("time_duration",time_duration)) time_duration = 5.0;

  if (!nh.getParam("use_sim",use_sim)) use_sim = true;

  std::cout << "=======================================\n";
  std::cout << "q1 = " << q1.t() << "\n";
  std::cout << "q2 = " << q2.t() << "\n";
  std::cout << "time_duration = " << time_duration << "\n";
  std::cout << "=======================================\n";

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  if (use_sim) bh282_robot.reset(new bhand_::Bh282SimRobot());
  else bh282_robot.reset(new bhand_::Bh282Robot());

  // ===========   Set Joints trajectory  =================
  jointsTrajectory(q1, time_duration);

  // // ===========   Joint Position Control  ================
  jointPositionControl(q2, time_duration);

  // ===========   Joint Velocity Control  ================
  jointVelocityControl(q1, time_duration);

  // ===========   FREEDIRVE  ================
  freedrive();

  return 0;
}
