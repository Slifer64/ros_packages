#include <iostream>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <ros/ros.h>
#include <armadillo>
#include <lwr4p/robot_arm.h>
#include <lwr4p/sim_robot.h>
#include <lwr4p/robot.h>
#include <misc/joint_state_publisher.h>

using namespace as64_;

double err_thres = 1e-2;

struct ExecArgs
{
  ExecArgs() {}
  ExecArgs(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
  {
    this->qT = qT;
    this->total_time = total_time;
    this->robot = robot;
  }
  arma::vec qT;
  double total_time;
  lwr4p_::RobotArm *robot;
};

void PRINT_INFO_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[34m" << "[RobotSim INFO]: " << msg << "\033[0m\n";
}

void PRINT_ERR_MSG(const std::string &msg)
{
  std::cerr << "\033[1m\033[31m" << "[RobotSim ERROR]: " << msg << "\033[0m\n";
}

void jointsTrajectory(const ExecArgs *args)
{
  arma::vec qT = args->qT;
  double total_time = args->total_time;
  lwr4p_::RobotArm *robot = args->robot;
  // ======================================================
  // ===========   Set Joints trajectory  =================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Joints Trajectory...");
  bool reached_target = robot->setJointsTrajectory(qT, total_time);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!\n");
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("==> " + robot->getErrMsg());
    robot->enable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void jointPositionControl(const ExecArgs *args)
{
  arma::vec qT = args->qT;
  double total_time = args->total_time;
  lwr4p_::RobotArm *robot = args->robot;

  // ======================================================
  // ===========   Joint Position Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_POS_CONTROL...");
  robot->setMode(lwr4p_::JOINT_POS_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint position control...");
  robot->update();
  double Ts = robot->getCtrlCycle();
  arma::vec q = robot->getJointsPosition();
  arma::vec q0 = q;
  double t = 0;
  while (arma::norm(qT-q)>err_thres && robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[0];
    robot->setJointsPosition(q_ref);
    robot->update();
    q = robot->getJointsPosition();
  }
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void jointVelocityControl(const ExecArgs *args)
{
  arma::vec qT = args->qT;
  double total_time = args->total_time;
  lwr4p_::RobotArm *robot = args->robot;

  // ======================================================
  // ===========   Joint Velocity Control  ================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to JOINT_VEL_CONTROL...");
  robot->setMode(lwr4p_::JOINT_VEL_CONTROL);
  PRINT_INFO_MSG("==> Moving with joint velocity control...");
  robot->update();
  double Ts = robot->getCtrlCycle();
  arma::vec q = robot->getJointsPosition();
  arma::vec q0 = q;
  double k_click = 0.1;
  double t = 0;
  while (arma::norm(qT-q)>err_thres && robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[0];
    arma::vec dq_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[1];
    robot->setJointsVelocity(dq_ref + k_click*(q_ref-q));
    robot->update();
    q = robot->getJointsPosition();
  }
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void cartesianVelocityControl(const ExecArgs *args)
{
  arma::vec qT = args->qT;
  double total_time = args->total_time;
  lwr4p_::RobotArm *robot = args->robot;

  // ==========================================================
  // ===========   Cartesian Velocity Control  ================
  // ==========================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to CART_VEL_CONTROL...");
  robot->setMode(lwr4p_::CART_VEL_CONTROL);
  PRINT_INFO_MSG("==> Moving with Cartesian velocity control...");
  robot->update();
  arma::vec q = robot->getJointsPosition();
  arma::vec q0 = q;
  arma::mat p0 = robot->getTaskPose(q0);
  arma::mat pT = robot->getTaskPose(qT);
  arma::mat p = robot->getTaskPose();
  double t = 0;
  double Ts = robot->getCtrlCycle();
  // Notice that if we were to check the joint position instead of the Cartesian we may never
  // reach qT, because due to pinv(J) we may arrive at pT with a different joint configuration.
  while (arma::norm(arma::vectorise(pT)-arma::vectorise(p))>err_thres && robot->isOk())
  {
    if (!ros::ok()) exit(-1);
    t += Ts;
    arma::vec q_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[0];
    // arma::vec dq_ref = lwr4p_::get5thOrder(t, q0, qT, total_time)[1];
    arma::vec v_click(6);
    arma::mat p_ref = robot->getTaskPose(q_ref);
    // position error
    v_click.subvec(0,2) = p_ref.submat(0,3,2,3) - p.submat(0,3,2,3);
    // orientation error (difference of quaternions vector parts)
    v_click.subvec(3,5) = lwr4p_::rotm2quat(p_ref.submat(0,0,2,2)).subvec(1,3) - lwr4p_::rotm2quat(p.submat(0,0,2,2)).subvec(1,3);
    // v_click *= 4.0;
    // arma::vec V = robot->getJacobian()*dq_ref + v_click;
    arma::vec V = v_click/Ts;
    robot->setTaskVelocity(V);
    robot->update();
    p = robot->getTaskPose();
  }
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
  else PRINT_INFO_MSG("==> Reached target pose!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void taskTrajectory(const ExecArgs *args)
{
  arma::vec qT = args->qT;
  double total_time = args->total_time;
  lwr4p_::RobotArm *robot = args->robot;

  // ======================================================
  // ===========   Set Task trajectory  =================
  // ======================================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Moving with Task Trajectory...");
  arma::mat target_pose = robot->getTaskPose(qT);
  bool reached_target = robot->setTaskTrajectory(target_pose, total_time);
  if (reached_target) PRINT_INFO_MSG("==> Reached target pose!");
  else  PRINT_ERR_MSG("==> Failed to reach target pose!");
  if (!robot->isOk())
  {
    PRINT_ERR_MSG("==> " + robot->getErrMsg());
    robot->enable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void freedrive(const ExecArgs *args)
{
  lwr4p_::RobotArm *robot = args->robot;

  // =========================================
  // ===========   FREEDIRVE  ================
  // =========================================
  std::cerr << "=====================================\n";
  PRINT_INFO_MSG("==> Setting the robot to FREEDRIVE...");
  robot->setMode(lwr4p_::FREEDRIVE);
  PRINT_INFO_MSG("==> The robot is in FREEDIRVE. Move it wherever you want. Press ctrl+C to exit...");

  while (ros::ok() && robot->isOk())
  {
    robot->update();
  }

  if (!robot->isOk())
  {
    PRINT_INFO_MSG("[ERROR]: " + robot->getErrMsg());
    robot->enable();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwr4p_robot_sim");
  ros::NodeHandle nh("~");

  int N_robots = 2;

  std::vector<arma::vec> q1[N_robots];
  std::vector<arma::vec> q2[N_robots];
  std::vector<double> time_duration[N_robots]; // sec
  std::vector<bool> use_sim[N_robots];

  std::vector<double> q_vec;

  if (nh.getParam("q1_1",q_vec)) q1[0] = q_vec;
  else  q1[0] = {-0.5, 0.6, 0.3, -1.5, 0, -0.2, 1.1};
  if (nh.getParam("q2_1",q_vec)) q2[0] = q_vec;
  else  q2[0] = {0.8, 0.9, -0.1, -1.3, -0.1, -1.2, -1.3};
  if (!nh.getParam("time_duration",time_duration)) throw std::ios_base::failure("Couldn't read time duration param.");

  if (nh.getParam("use_sim",use_sim)) use_sim = true;

  std::cout << "=======================================\n";
  std::cout << "q1 = " << q1.t() << "\n";
  std::cout << "q2 = " << q2.t() << "\n";
  std::cout << "time_duration = " << time_duration << "\n";
  std::cout << "use_sim = " << (use_sim?"true":"false") << "\n";
  std::cout << "=======================================\n";

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // initialize robot 1
  std::shared_ptr<lwr4p_::RobotArm> lwr4p_robot;
  if (use_sim) lwr4p_robot.reset(new lwr4p_::SimRobot());
  else lwr4p_robot.reset(new lwr4p_::Robot());
  lwr4p_robot->setJointLimitCheck(true);
  lwr4p_robot->setSingularityCheck(true);
  // lwr4p_robot->setSingularityThreshold(8e-3);
  // lwr4p_robot->readWrenchFromTopic(true, "/wrench");

  std::thread thr; // for running the robot control action
  ExecArgs args; // arguments for each action

  // initialize joint state publisher
  as64_::misc_::JointStatePublisher jState_pub;
  jState_pub.setPublishCycle(0.01);
  std::string publish_states_topic;
  nh.getParam("publish_states_topic",publish_states_topic);
  jState_pub.setPublishTopic(publish_states_topic);
  // jState_pub.setPublishTopic("/robot_joint_states");
  jState_pub.addFun(&lwr4p_::SimRobot::addJointState, lwr4p_robot.get());

  jState_pub.start(); // launches joint states publisher thread

  // ===========   Set Joints trajectory  =================
  args = ExecArgs(q1, time_duration, lwr4p_robot.get());
  thr = std::thread(jointsTrajectory, &args);
  // do other stuff ...
  thr.join();
  // or: jointsTrajectory(&args);

  // ===========   Joint Position Control  ================
  args = ExecArgs(q2, time_duration, lwr4p_robot.get());
  thr = std::thread(jointPositionControl, &args);
  // do other stuff ...
  thr.join();
  // or: jointPositionControl(&args);

  // ===========   Joint Velocity Control  ================
  args = ExecArgs(q1, time_duration, lwr4p_robot.get());
  thr = std::thread(jointVelocityControl, &args);
  // do other stuff ...
  thr.join();
  // or: jointVelocityControl(&args);

  // ===========   Cartesian Velocity Control  ================
  args = ExecArgs(q2, time_duration, lwr4p_robot.get());
  thr = std::thread(cartesianVelocityControl, &args);
  // do other stuff ...
  thr.join();
  // or: cartesianVelocityControl(&args);

  // ===========   Set Tasktrajectory  =================
  args = ExecArgs(q1, time_duration, lwr4p_robot.get());
  thr = std::thread(taskTrajectory, &args);
  // do other stuff ...
  thr.join();
  // or: taskTrajectory(&args);

  // ===========   FREEDIRVE  ================
  if (use_sim) jState_pub.stop(); // stop publishing in freedrive when using SimRobot
  args = ExecArgs(q1, time_duration, lwr4p_robot.get());
  thr = std::thread(freedrive, &args);
  // do other stuff ...
  thr.join();
  // or: freedrive(&args);

  return 0;
}
