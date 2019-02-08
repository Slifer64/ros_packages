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
#include <misc_lib/joint_state_publisher.h>
#include <io_lib/xml_parser.h>

#include <ros/ros.h>
#include <ros/package.h>

using namespace as64_;

double err_thres = 1e-2;

struct ExecArgs
{
  ExecArgs() {}
  ExecArgs(const arma::vec &q1, const arma::vec &q2, double total_time, lwr4p_::RobotArm *robot)
  {
    this->q1 = q1;
    this->q2 = q2;
    this->total_time = total_time;
    this->robot = robot;
  }
  arma::vec q1;
  arma::vec q2;
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

void jointsTrajectory(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{

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

void jointPositionControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{

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

void jointVelocityControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{

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

void cartesianVelocityControl(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{
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

void taskTrajectory(const arma::vec &qT, double total_time, lwr4p_::RobotArm *robot)
{

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

void freedrive(lwr4p_::RobotArm *robot)
{

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

void robotRun(ExecArgs *args)
{
  arma::vec q1 = args->q1;
  arma::vec q2 = args->q2;
  double time_duration = args->total_time;
  lwr4p_::RobotArm *lwr4p_robot = args->robot;

  // ===========   Set Joints trajectory  =================
  jointsTrajectory(q1, time_duration, lwr4p_robot);

  // ===========   Joint Position Control  ================
  jointPositionControl(q2, time_duration, lwr4p_robot);

  // ===========   Joint Velocity Control  ================
  jointVelocityControl(q1, time_duration, lwr4p_robot);

  // ===========   Cartesian Velocity Control  ================
  cartesianVelocityControl(q2, time_duration, lwr4p_robot);

  // ===========   Set Tasktrajectory  =================
  taskTrajectory(q1, time_duration, lwr4p_robot);

  // ===========   FREEDIRVE  ================
  // sync all robots before stop publishing...
  // if (use_sim) jState_pub.stop(); // stop publishing in freedrive when using SimRobot
  freedrive(lwr4p_robot);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_lwr4p_robot_test");
  ros::NodeHandle nh("~");

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // ============================================
  // =========== Parse values  ==================
  // ============================================

  // initialize parser providing the path to the config file
  std::string confi_file_path = ros::package::getPath("robot_test") + "/config/multi_lwr4p_robots_test.yaml";
  as64_::io_::XmlParser parser(confi_file_path);

  std::string robot_description_name;
  std::vector<std::string> base_link;
  std::vector<std::string> tool_link;
  std::vector<double> ctrl_cycle;

  arma::mat q1;
  arma::mat q2;
  std::vector<double> time_duration;
  std::vector<bool> use_sim;

  if (!parser.getParam("robot_description_name",robot_description_name)) throw std::ios_base::failure("Failed to read parameter \"robot_description_name\".");
  if (!parser.getParam("base_link",base_link)) throw std::ios_base::failure("Failed to read parameter \"base_link\".");
  if (!parser.getParam("tool_link",tool_link)) throw std::ios_base::failure("Failed to read parameter \"tool_link\".");
  if (!parser.getParam("ctrl_cycle",ctrl_cycle)) throw std::ios_base::failure("Failed to read parameter \"ctrl_cycle\".");

  if (!parser.getParam("q1",q1)) throw std::ios_base::failure("Failed to read parameter \"q1\".");
  if (!parser.getParam("q2",q2)) throw std::ios_base::failure("Failed to read parameter \"q2\".");
  if (!parser.getParam("time_duration",time_duration)) throw std::ios_base::failure("Failed to read parameter \"time_duration\".");
  if (!parser.getParam("use_sim",use_sim)) throw std::ios_base::failure("Failed to read parameter \"use_sim\".");

  int N_robots = base_link.size();

  // =========================================
  // =========== initialize robots ===========
  // =========================================
  std::vector<std::shared_ptr<lwr4p_::RobotArm>> lwr4p_robot(N_robots);
  for (int i=0; i<N_robots; i++)
  {
    if (use_sim[i]) lwr4p_robot[i].reset(new lwr4p_::SimRobot(robot_description_name, base_link[i], tool_link[i], ctrl_cycle[i]));
    else lwr4p_robot[i].reset(new lwr4p_::Robot(robot_description_name, base_link[i], tool_link[i], ctrl_cycle[i]));
    lwr4p_robot[i]->setJointLimitCheck(true);
    lwr4p_robot[i]->setSingularityCheck(true);
    // lwr4p_robot[i]->setSingularityThreshold(8e-3);
    // lwr4p_robot[i]->readWrenchFromTopic(true, "/wrench");
  }

  // ========================================================
  // =========== initialize joint state publisher ===========
  // ========================================================
  as64_::misc_::JointStatePublisher jState_pub;
  jState_pub.setPublishCycle(0.0333); // 30 Hz
  std::string publish_states_topic;
  if (!nh.getParam("publish_states_topic",publish_states_topic)) throw std::ios_base::failure("Failed to read parameter \"publish_states_topic\".");
  jState_pub.setPublishTopic(publish_states_topic);

  std::cerr << "=========> publish_states_topic = " << publish_states_topic << "\n";
  // jState_pub.setPublishTopic("/robot_joint_states");
  for (int i=0; i<N_robots; i++) jState_pub.addFun(&lwr4p_::SimRobot::addJointState, lwr4p_robot[i].get());

  jState_pub.start(); // launches joint states publisher thread

  std::vector<std::thread> robot_run_thead(N_robots);
  std::vector<std::shared_ptr<ExecArgs>> args(N_robots);
  for (int i=0; i<N_robots; i++)
  {
    args[i].reset(new ExecArgs(q1.row(i).t(), q2.row(i).t(), time_duration[i], lwr4p_robot[i].get()));
    robot_run_thead[i] = std::thread(robotRun, args[i].get());
  }

  for (int i=0; i<N_robots; i++)
  {
    if (robot_run_thead[i].joinable()) robot_run_thead[i].join();
  }

  jState_pub.stop();

  return 0;
}
