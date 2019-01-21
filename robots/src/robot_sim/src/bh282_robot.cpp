#include <robot_sim/bh282_robot.h>

namespace as64_
{

namespace robot_sim_
{

Bh282Robot::Bh282Robot()
{
  if (!node.getParam("/bh282_robot/base_frame",base_link))
  {
    throw std::runtime_error("Failed to load parameter \"/bh282_robot/base_frame\" ...\n");
  }

  if (!node.getParam("/bh282_robot/tool_frame",tool_link))
  {
    throw std::runtime_error("Failed to load parameter \"/bh282_robot/tool_frame\" ...\n");
  }

  if (!node.getParam("/bh282_robot/check_limits",check_limits))
  {
    check_limits = false;
  }

  if (!node.getParam("/bh282_robot/check_singularity",check_singularity))
  {
    check_singularity = false;
  }

  if (!node.getParam("/bh282_robot/ctrl_cycle",ctrl_cycle))
  {
    ctrl_cycle = 0.01;
  }

  if (!node.getParam("/bh282_robot/wrench_topic",wrench_topic))
  {
    int n = tool_link.size();
    wrench_topic.resize(n);
    for (int i=0;i<n;i++) wrench_topic[i] = "";
    // read_wrench_from_topic = true;
  }

  std::string robot_description_name;
  if (!node.getParam("/bh282_robot/robot_description_name",robot_description_name))
  {
    throw std::runtime_error("Failed to load parameter \"/bh282_robot/robot_description_name\" ...\n");
  }

  //std::string urdf_file_path = ros::package::getPath("robot_sim") + "/urdf/bh282_robot.urdf";

  if (!urdf_model.initParam(robot_description_name.c_str()))
  // if (!urdf_model.initFile(urdf_file_path.c_str()))
  {
    throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_description_name + "\"...\n");
  }

  init();
}

void Bh282Robot::getChainFingerInd(Bh282Robot::JointName jn, int &chain_ind, int &joint_ind) const
{
  if (jn == Bh282Robot::PACK)
  {
    chain_ind = 0; // pack joint is in the first chain
    joint_ind = 0; // and is the first joint of that chain
  }
  else if (jn == Bh282Robot::FING1)
  {
    chain_ind = 0; // fing1 joint is in the first chain
    joint_ind = 1; // and is the second element of that chain
  }
  else
  {
    // the chains for fing2 and fing3 have only the corresponding finger joint
    chain_ind = (int)jn - 1;
    joint_ind = 0;
  }
}

void Bh282Robot::setJointPos(double pos, Bh282Robot::JointName jn)
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  arma::vec j_pos = getJointsPosition(chain_ind);
  j_pos(joint_ind) = pos;
  setJointsPosition(j_pos, chain_ind);
}

void Bh282Robot::setJointVel(double vel, Bh282Robot::JointName jn)
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  arma::vec j_vel = arma::vec().zeros(fingers[chain_ind]->getNumJoints());
  j_vel(joint_ind) = vel;
  setJointsVelocity(j_vel, chain_ind);
}

bool Bh282Robot::setJointsTraj(arma::vec j_target, double duration, const std::vector<Bh282Robot::JointName> &jn)
{
  // keep last known robot mode
  robot_sim_::Mode prev_mode = getMode();

  // initialize position
  arma::vec q0(jn.size());
  for (int i=0; i<q0.size(); i++) q0(i) = getJointPos(jn[i]);
  arma::vec qref = q0;

  // initalize time
  double t = 0.0;
  double Ts = getCtrlCycle();

  // start conttroller
  setMode(robot_sim_::Mode::JOINT_POS_CONTROL);
  while (isOk() && getMode()!=robot_sim_::Mode::IDLE && t<duration)
  {
    // std::cout << "t = " << t  << " sec\n";
    update();
    t += Ts;
    qref = get5thOrder(t, q0, j_target, duration)[0];
    for (int i=0; i<qref.size(); i++) setJointPos(qref(i),jn[i]);
  }

  bool reached_target = t>=duration;

  if (isOk() && getMode()==robot_sim_::Mode::JOINT_POS_CONTROL) setMode(prev_mode);

  return reached_target;
}

double Bh282Robot::getJointPos(Bh282Robot::JointName jn) const
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  return (getJointsPosition(chain_ind))(joint_ind);
}

double Bh282Robot::getJointVel(Bh282Robot::JointName jn) const
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  return (getJointsVelocity(chain_ind))(joint_ind);
}

}; // namespace robot_sim_

}; // namespace as64_
