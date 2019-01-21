#include <robot_sim/robot_chain.h>

#include <stdexcept>
#include <sstream>
#include <chrono>
#include <map>
#include <stack>

#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>

// #define RESET   "\033[0m"
// #define BLACK   "\033[30m"  /* Black */
// #define RED     "\033[31m"  /* Red */
// #define GREEN   "\033[32m"  /* Green */
// #define YELLOW  "\033[33m"  /* Yellow */
// #define BLUE    "\033[34m"  /* Blue */
// #define MAGENTA "\033[35m"  /* Magenta */
// #define CYAN    "\033[36m"  /* Cyan */
// #define WHITE   "\033[37m"  /* White */
// #define BOLD    "\033[1m"   /* Bold */

namespace as64_
{

namespace robot_sim_
{

RobotChain::RobotChain(urdf::Model &urdf_model,
  const std::string &base_link, const std::string &tool_link,
  double ctrl_cycle, bool check_limits, bool check_singularity,
  const std::string &wrench_topic)
{
  this->urdf_model = urdf_model;
  this->base_link_name = base_link;
  this->tool_link_name = tool_link;
  this->ctrl_cycle = ctrl_cycle;
  this->check_limits = check_limits;
  this->check_singularity = check_singularity;
  this->wrench_topic = wrench_topic;
  this->read_wrench_from_topic = wrench_topic.compare("");

  init();
}

RobotChain::RobotChain()
{
  // if (!node.getParam("/robot_sim/base_frame",base_link_name))
  // {
  //   throw std::runtime_error("Failed to load parameter \"/robot_chain/base_frame\" ...\n");
  // }
  //
  // if (!node.getParam("/robot_chain/tool_frame",tool_link_name))
  // {
  //   throw std::runtime_error("Failed to load parameter \"/robot_chain/tool_frame\" ...\n");
  // }
  //
  // if (!node.getParam("/robot_chain/check_limits",check_limits))
  // {
  //   check_limits = false;
  // }
  //
  // if (!node.getParam("/robot_chain/check_singularity",check_singularity))
  // {
  //   check_singularity = false;
  // }
  //
  // if (!node.getParam("/robot_chain/ctrl_cycle",ctrl_cycle))
  // {
  //   ctrl_cycle = 0.01;
  // }
  //
  // if (node.getParam("/robot_chain/wrench_topic",wrench_topic))
  // {
  //   read_wrench_from_topic = true;
  // }
  //
  // std::string robot_description_name;
  // if (!node.getParam("/robot_chain/robot_description_name",robot_description_name))
  // {
  //   throw std::runtime_error("Failed to load parameter \"/robot_chain/robot_description_name\" ...\n");
  // }
  //
  // std::string urdf_file_path = ros::package::getPath("robot_sim") + "/urdf/lwr4p_robot.urdf";
  //
  // if (!urdf_model.initParam(robot_description_name.c_str()))
  // // if (!urdf_model.initFile(urdf_file_path.c_str()))
  // {
  //   throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_description_name + "\"...\n");
  // }
  //
  // init();

}

RobotChain::~RobotChain() {}

void RobotChain::init()
{
  mode_name.resize(6);
  mode_name[0] = "IDLE";
  mode_name[1] = "FREEDRIVE";
  mode_name[2] = "JOINT_POS_CONTROL";
  mode_name[3] = "JOINT_VEL_CONTROL";
  mode_name[4] = "CART_VEL_CONTROL";
  mode_name[5] = "PROTECTIVE_STOP";

  mode = robot_sim_::Mode::IDLE;

  SINGULARITY_THRES = 8e-3;


  // find base_link and tool_link
  bool found_base_link = false;
  bool found_tool_link = false;
  boost::shared_ptr<const urdf::Link> base_link;
  boost::shared_ptr<const urdf::Link> tool_link;
  std::stack<boost::shared_ptr<const urdf::Link>> link_stack;
  link_stack.push(urdf_model.getRoot());
  while (!link_stack.empty())
  {
    auto link = link_stack.top();
    link_stack.pop();

    if (base_link_name.compare(link->name) == 0)
    {
      base_link = link;
      found_base_link = true;
    }

    if (tool_link_name.compare(link->name) == 0)
    {
      tool_link = link;
      found_tool_link = true;
    }

    for (int i=0;i<link->child_links.size();i++) link_stack.push(link->child_links[i]);
    // for (int i=0;i<link->child_joints.size();i++) _joints.push_back(link->child_joints[i]);
  }

  if (!found_base_link)
    throw std::runtime_error("Couldn't find specified base link \"" + base_link_name + "\" in the robot urdf model...\n");

  if (!found_tool_link)
    throw std::runtime_error("Couldn't find specified tool link \"" + tool_link_name + "\" in the robot urdf model...\n");

  // find all links in the chain from tool_link to base_link
  std::vector<boost::shared_ptr<const urdf::Link>> chain_links;
  auto link = tool_link;
  while (link->name.compare(base_link->name))
  {
    chain_links.push_back(link);
    link = link->getParent();
  }
  chain_links.push_back(base_link);

  // parse all joints for each link starting from base_link
  for (int i=chain_links.size()-1; i>0; i--)
  {
    link = chain_links[i];
    auto next_link = chain_links[i-1];

    for (int i=0;i<link->child_joints.size();i++)
    {
      auto joint = link->child_joints[i];
      auto jtype = joint->type;

      if (jtype==urdf::Joint::FIXED || jtype==urdf::Joint::FLOATING) continue;

      if (joint->mimic) continue;

      if (joint->child_link_name.compare(next_link->name)) continue;

      joint_names.push_back(joint->name);

      if (jtype==urdf::Joint::CONTINUOUS)
      {
        joint_pos_lower_lim.push_back(-M_PI);
        joint_pos_upper_lim.push_back(M_PI);
      }
      else
      {
        joint_pos_lower_lim.push_back(joint->limits->lower);
        joint_pos_upper_lim.push_back(joint->limits->upper);
      }

      effort_lim.push_back(joint->limits->effort);
      joint_vel_lim.push_back(joint->limits->velocity);
    }
  }

  // create KDL::Chain and forward/inverse kinematics and Jacobian solvers
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(urdf_model, tree);

  if (!tree.getChain(base_link_name, tool_link_name, chain))
  {
    throw std::runtime_error("Failed to create kdl chain from " + base_link_name + " to " + tool_link_name + " ...\n");
  }
  else
  {
    fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    ik_vel_solver.reset(new KDL::ChainIkSolverVel_pinv(chain));
    ik_solver.reset(new KDL::ChainIkSolverPos_NR(chain,*fk_solver,*ik_vel_solver,200,1e-6));
  }

  // preallocate space for all states
  N_JOINTS = joint_names.size();
  joint_pos.zeros(N_JOINTS);
  joint_prev_pos.zeros(N_JOINTS);
  joint_vel.zeros(N_JOINTS);
  joint_torques.zeros(N_JOINTS);
  pose.resize(4,4);
  Jrobot.resize(6,N_JOINTS);
  Jee.resize(6, N_JOINTS);
  Fext.zeros(6);

  // subscribe to topics
  pub_topic = "/robot_sim_joint_states";
  jState_pub = node.advertise<sensor_msgs::JointState>(pub_topic, 1);

  jState_sub = node.subscribe("/joint_states", 1, &RobotChain::jStateSubCallback, this);

  if (read_wrench_from_topic) wrench_sub = node.subscribe(wrench_topic.c_str(), 1, &RobotChain::readWrenchCallback, this);

  // start control-cycle timer
  update_time = (int)(getCtrlCycle()*1e9); // use nanosec for greater precision
  timer.start();

  // print chain info
  // std::cout << "==============================\n";
  // std::cout << "Chain joints:\n";
  // for (int i=0;i<joint_names.size();i++)
  // {
  //   std::cout << joint_names[i] << ": limits: [" << joint_pos_lower_lim[i] << ", " << joint_pos_upper_lim[i] << "]\n";
  // }
  // std::cout << "\n==============================\n";
}

void RobotChain::printRobotInfo() const
{
  for (int i=0;i<joint_names.size();i++)
  {
    std::cout << "joint" << i+1 << ": " << joint_names[i] << "\n";
    std::cout << "pos_limits: [" << joint_pos_lower_lim[i] << ", " << joint_pos_upper_lim[i] << "] rad\n";
    std::cout << "vel limit: " << joint_vel_lim[i] << " rad/s\n";
  }
}

bool RobotChain::isOk() const
{
  return getMode()!=robot_sim_::Mode::PROTECTIVE_STOP;
}

void RobotChain::enable()
{
  setMode(robot_sim_::Mode::IDLE);
}

std::string RobotChain::getErrMsg() const
{
  return err_msg;
}

void RobotChain::updateState()
{
  std::unique_lock<std::mutex> lck(robot_state_mtx);

  ros::spinOnce();

  joint_vel = (joint_pos - joint_prev_pos) / ctrl_cycle;

  pose = getTaskPose(joint_pos);
  Jrobot = getJacobian(joint_pos);

  arma::mat R = pose.submat(0,0,2,2);
  Jee.submat(0,0,2,N_JOINTS-1) = R * Jrobot.submat(0,0,2,N_JOINTS-1);
  Jee.submat(3,0,5,N_JOINTS-1) = R * Jrobot.submat(3,0,5,N_JOINTS-1);

  joint_torques = Jrobot.t() * Fext;
}

void RobotChain::publishState()
{
  if (getMode() == robot_sim_::Mode::FREEDRIVE) return;

  sensor_msgs::JointState joint_state_msg;
  addJointState(joint_state_msg);
  jState_pub.publish(joint_state_msg);
}

void RobotChain::addJointState(sensor_msgs::JointState &joint_state_msg)
{
  std::unique_lock<std::mutex> lck(robot_state_mtx);

  for (int i=0;i<N_JOINTS;i++)
  {
    joint_state_msg.name.push_back(joint_names[i]);
    joint_state_msg.position.push_back(joint_pos(i));
    joint_state_msg.velocity.push_back(joint_vel(i));
    joint_state_msg.effort.push_back(0.0);
  }
}

void RobotChain::update()
{
  updateState();

  publishState();

  // wait for timecycle to complete
  unsigned long elaps_time = timer.elapsedNanoSec();
  if (elaps_time < update_time)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(update_time-elaps_time));
  }
  timer.start(); // restart time cycle
}

double RobotChain::getCtrlCycle() const
{
  return ctrl_cycle;
}

void RobotChain::setMode(const robot_sim_::Mode &m)
{
  if (getMode() == m) return;

  if (m == robot_sim_::Mode::PROTECTIVE_STOP) protectiveStop();
  else
  {
    // if (getMode()!=robot_sim_::Mode::IDLE && getMode()!=robot_sim_::Mode::PROTECTIVE_STOP)
    mode = m;
    stop();
    if (getMode() == robot_sim_::Mode::PROTECTIVE_STOP) return;
    // mode = m;
    // print_info_msg("Mode changed to \"" + getModeName() + "\"\n");
  }
}

robot_sim_::Mode RobotChain::getMode() const
{
  return mode;
}

std::string RobotChain::getModeName() const
{
  return mode_name[getMode()];
}

void RobotChain::stop()
{
  update();
  arma::vec q_current = getJointsPosition();
  setJointsPositionHelper(q_current);
  joint_prev_pos = joint_pos;
  update();
  // mode = robot_sim_::Mode::IDLE;
  // print_info_msg("Mode changed to \"" + getModeName() + "\"\n");
}

void RobotChain::protectiveStop()
{
  joint_prev_pos = joint_pos;
  update();
  mode = robot_sim_::Mode::PROTECTIVE_STOP;
  print_warn_msg("Mode changed to \"" + getModeName() + "\"\n");
}

int RobotChain::getNumJoints() const
{
  return N_JOINTS;
}

void RobotChain::setJointsPosition(const arma::vec &j_pos)
{
  err_msg = ""; // clear the error msg
  if (getMode() != robot_sim_::Mode::JOINT_POS_CONTROL)
  {
    print_warn_msg("Cannot set joints position. Current mode is \"" + getModeName() + "\"\n");
    return;
  }

  setJointsPositionHelper(j_pos);
}

void RobotChain::setJointsVelocity(const arma::vec &j_vel)
{
  err_msg = ""; // clear the error msg
  if (getMode() != robot_sim_::Mode::JOINT_VEL_CONTROL)
  {
    print_warn_msg("Cannot set joints velocity. Current mode is \"" + getModeName() + "\"\n");
    return;
  }

  setJointsVelocityHelper(j_vel);
}

void RobotChain::setTaskVelocity(const arma::vec &task_vel)
{
  err_msg = ""; // clear the error msg
  if (getMode() != robot_sim_::Mode::CART_VEL_CONTROL)
  {
    print_warn_msg("Cannot set task velocity. Current mode is \"" + getModeName() + "\"\n");
    return;
  }
  setTaskVelocityHelper(task_vel);
}

void RobotChain::setJointsPositionHelper(const arma::vec &j_pos)
{
  arma::vec dj_pos = (joint_pos - j_pos) / ctrl_cycle;

  if (check_limits)
  {
    if (!checkJointPosLimits(j_pos)) return;
    if (!checkJointVelLimits(dj_pos)) return;
  }

  if (check_singularity)
  {
    if (!checkSingularity()) return;
  }

  joint_prev_pos = joint_pos;
  joint_pos = j_pos;
}

void RobotChain::setJointsVelocityHelper(const arma::vec &j_vel)
{
  setJointsPositionHelper(joint_pos + j_vel*ctrl_cycle);
}

void RobotChain::setTaskVelocityHelper(const arma::vec &task_vel)
{
  setJointsVelocityHelper(arma::solve(Jrobot,task_vel));
}

bool RobotChain::setJointsTrajectory(const arma::vec &j_targ, double duration)
{
  // keep last known robot mode
  robot_sim_::Mode prev_mode = getMode();

  // initialize position
  arma::vec q0 = getJointsPosition();
  arma::vec qref = q0;

  // initalize time
  double t = 0.0;
  double Ts = getCtrlCycle();

  // start conttroller
  setMode(robot_sim_::Mode::JOINT_POS_CONTROL);
  while (isOk() && getMode()!=robot_sim_::Mode::IDLE && t<duration)
  {
    update();
    t += Ts;
    qref = get5thOrder(t, q0, j_targ, duration)[0];
    setJointsPosition(qref);
  }

  bool reached_target = t>=duration;

  if (isOk() && getMode()==robot_sim_::Mode::JOINT_POS_CONTROL) setMode(prev_mode);

  return reached_target;
}

bool RobotChain::setTaskTrajectory(const arma::mat &target_pose, double duration)
{
  bool found_solution = false;
  // arma::vec j_pos0 = joint_pos;
  arma::vec j_pos0 = arma::vec().zeros(N_JOINTS);

  // find an initial joint configuration that yiels a pose close to target_pose
  for (int i=N_JOINTS-1; i>-1; i--)
  {
    int n_div = 8;
    arma::vec jp = arma::linspace<arma::vec>(joint_pos_lower_lim[i], joint_pos_upper_lim[i], n_div);
    jp = jp.subvec(1,n_div-2);
    arma::vec j_pos = j_pos0;
    for (int j=0; j<jp.size() ; j++)
    {
      j_pos[i] = jp[j];
      arma::mat pose0 = getTaskPose(j_pos0);
      arma::mat pose = getTaskPose(j_pos);
      double e0 = arma::norm(robot_sim_::posError(pose0.submat(0,3,2,3),target_pose.submat(0,3,2,3))) + arma::norm(robot_sim_::orientError(pose0.submat(0,0,2,2),target_pose.submat(0,0,2,2)));
      double e = arma::norm(robot_sim_::posError(pose.submat(0,3,2,3),target_pose.submat(0,3,2,3))) + arma::norm(robot_sim_::orientError(pose.submat(0,0,2,2),target_pose.submat(0,0,2,2)));
      if (e < e0) j_pos0 = j_pos;
    }
  }

  arma::vec j_target = getJointsPosition(target_pose, j_pos0, &found_solution);

  if (found_solution)
  {
    // check if the found solution respects the joint limits
    for (int i=0; i<j_target.size(); i++)
    {
      if (j_target(i)>joint_pos_upper_lim[i] || j_target(i)<joint_pos_lower_lim[i]) return false;
    }
    return setJointsTrajectory(j_target, duration);
  }


  return false;
}

arma::vec RobotChain::getJointsPosition() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_pos;
}

arma::vec RobotChain::getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution) const
{
  KDL::JntArray jnt(N_JOINTS);
  KDL::JntArray jnt0(N_JOINTS);

  for (int i=0;i<N_JOINTS;i++) jnt0(i) = q0(i);

  KDL::Frame kdl_pose;
  for (int i=0;i<3;i++)
  {
    kdl_pose.p[i] = pose(i,3);
    for (int j=0;j<3;j++) kdl_pose.M(i,j) = pose(i,j);
  }

  int ret = ik_solver->CartToJnt(jnt0,kdl_pose,jnt);

  if (found_solution) *found_solution = ret >= 0;

  arma::vec q = arma::vec().zeros(N_JOINTS);

  if (ret>=0)
  {
    for (int i=0;i<N_JOINTS;i++) q(i) = jnt(i);
  }

  return q;
}

arma::vec RobotChain::getJointsVelocity() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_vel;
}

arma::mat RobotChain::getTaskPose() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return pose;
}

arma::mat RobotChain::getTaskPose(const arma::vec &j_pos) const
{
  arma::mat task_pose(4,4);

  KDL::JntArray jnt(N_JOINTS);
  for (int i=0;i<N_JOINTS;i++) jnt(i) = j_pos(i);

  KDL::Frame fk;
  fk_solver->JntToCart(jnt, fk);
  for (int i=0;i<3;i++)
  {
    for (int j=0;j<4;j++) task_pose(i,j) = fk(i,j);
  }
  task_pose.row(3) = arma::rowvec({0,0,0,1});

  return task_pose;
}

arma::vec RobotChain::getTaskPosition() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return pose.submat(0,3,2,3);
}

arma::vec RobotChain::getTaskOrientation() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  arma::mat R = pose.submat(0,0,2,2);
  arma::vec quat = rotm2quat(R);
  return quat;
}

arma::mat RobotChain::getJacobian() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return Jrobot;
}

arma::mat RobotChain::getJacobian(const arma::vec j_pos) const
{
  KDL::JntArray jnt(N_JOINTS);
  for (int i=0;i<N_JOINTS;i++) jnt(i) = j_pos(i);

  KDL::Jacobian J(N_JOINTS);
  jac_solver->JntToJac(jnt, J);
  arma::mat Jac(6, N_JOINTS);
  for (int i=0;i<Jrobot.n_rows;i++)
  {
    for (int j=0;j<Jrobot.n_cols;j++) Jac(i,j) = J(i,j);
  }

  return Jac;
}

arma::mat RobotChain::getEEJacobian() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return Jee;
}

arma::vec RobotChain::getJointTorques() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_torques;
}

arma::vec RobotChain::getExternalForce() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return Fext;
}

bool RobotChain::checkJointPosLimits(const arma::vec &j_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (j_pos(i)>joint_pos_upper_lim[i] || j_pos(i)<joint_pos_lower_lim[i])
    {
      std::ostringstream out;
      out << joint_names[i] << ": position limit reached: " << j_pos(i) << " rad\n";
      err_msg = out.str();
      // print_err_msg(err_msg);
      setMode(robot_sim_::Mode::PROTECTIVE_STOP);
      return false;
    }
  }

  return true;
}

bool RobotChain::checkJointVelLimits(const arma::vec &dj_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (std::fabs(dj_pos(i))>joint_vel_lim[i])
    {
      std::ostringstream out;
      out << joint_names[i] << ": velocity limit reached: " << dj_pos(i) << " rad/s\n";
      err_msg = out.str();
      // print_err_msg(err_msg);
      setMode(robot_sim_::Mode::PROTECTIVE_STOP);
      return false;
    }
  }

  return true;
}

bool RobotChain::checkSingularity()
{
  if (getMode()==robot_sim_::Mode::IDLE ||
      getMode()==robot_sim_::Mode::FREEDRIVE ||
      getMode()==robot_sim_::Mode::JOINT_POS_CONTROL ||
      getMode()==robot_sim_::Mode::JOINT_VEL_CONTROL) return true;

  bool singularity_reached = false;

  arma::vec eigval; // = arma::eig_gen(Jrobot);
  arma::svd(eigval, Jrobot);

  if (arma::min(arma::abs(eigval)) < SINGULARITY_THRES) singularity_reached = true;

  if (singularity_reached)
  {
    err_msg = "Singularity reached!\n";
    // print_err_msg(err_msg);
    setMode(robot_sim_::Mode::PROTECTIVE_STOP);
    return false;
  }

  return true;
}

void RobotChain::jStateSubCallback(const sensor_msgs::JointState::ConstPtr& j_state)
{
  if (getMode() != robot_sim_::Mode::FREEDRIVE) return;

  std::map<std::string, double> j_map;

  for (int i=0;i<j_state->name.size();i++)
    j_map.insert( std::pair<std::string,double>(j_state->name[i], j_state->position[i]) );

  for (int i=0;i<joint_names.size();i++)
    joint_pos[i] = j_map[joint_names[i]];
  // joint_vel = j_state->velocity;
  // joint_torques = j_state->effort;
}

void RobotChain::readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr)
{
  if (!read_wrench_from_topic) return;

  Fext(0) = wrench_ptr->wrench.force.x;
  Fext(1) = wrench_ptr->wrench.force.y;
  Fext(2) = wrench_ptr->wrench.force.z;

  Fext(3) = wrench_ptr->wrench.torque.x;
  Fext(4) = wrench_ptr->wrench.torque.y;
  Fext(5) = wrench_ptr->wrench.torque.z;
}

}; // namespace robot_sim_

}; // namespace as64_
