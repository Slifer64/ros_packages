#include <bhand/bh282_robot.h>

namespace as64_
{

namespace bhand_
{

Bh282Robot::Bh282Robot(urdf::Model &urdf_model, const std::vector<std::string> &base_link,
                      const std::vector<std::string> &tool_link, double ctrl_cycle)
                      :RobotHand(urdf_model, base_link, tool_link, ctrl_cycle)
{}

Bh282Robot::Bh282Robot(const std::string &robot_desc_param, const std::vector<std::string> &base_link,
                      const std::vector<std::string> &tool_link, double ctrl_cycle)
                      :RobotHand(urdf_model, base_link, tool_link, ctrl_cycle)
{}

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

  if (!node.getParam("/bh282_robot/publish_joint_state_topic",pub_state_topic))
  {
    pub_state_topic = "/robot_joint_states";
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

  //std::string urdf_file_path = ros::package::getPath("bhand") + "/urdf/bh282_robot.urdf";

  if (!urdf_model.initParam(robot_description_name.c_str()))
  // if (!urdf_model.initFile(urdf_file_path.c_str()))
  {
    throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_description_name + "\"...\n");
  }

  hw_i.initialize("BH8-280", true);

  init();

  k_click = 10.0;
}

Bh282Robot::~Bh282Robot()
{
  hw_i.terminate();
}

void Bh282Robot::update()
{
  hw_i.RTUpdate();

  checkJointPosDeviationError();

  if (getMode() == bhand_::FREEDRIVE) updateFreedrive();

  updateState(); // update the state

  publishState(); // publish joint state
}

void Bh282Robot::setMode(const bhand_::Mode &m)
{
  for (int i=0;i<N_fingers;i++)
  {
    fingers[i]->setMode(m);
    if (fingers[i]->getMode() != m) // error occured
    {
      // set all fingers to PROTECTIVE_STOP and return
      for (int j=0; j<N_fingers; j++) fingers[i]->setMode(bhand_::PROTECTIVE_STOP);
      mode = bhand_::PROTECTIVE_STOP;
      hw_i.setMode(BhandHWInterface::IDLE);
      return;
    }
  }
  mode = m;

  if (mode==bhand_::IDLE)
  {
    hw_i.setMode(BhandHWInterface::IDLE);
  }
  else if (mode==bhand_::FREEDRIVE)
  {
    initFreedrive();
    hw_i.setMode(BhandHWInterface::JOINT_VEL_CONTROL);
  }
  else hw_i.setMode(BhandHWInterface::JOINT_VEL_CONTROL);
}

void Bh282Robot::setJointPosition(double pos, bhand_::JointName jn)
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  arma::vec j_pos = fingers[chain_ind]->getJointsPosition();
  arma::vec j_pos_prev = j_pos;
  j_pos(joint_ind) = pos;
  fingers[chain_ind]->setJointPosition(j_pos);

  if (!fingers[chain_ind]->isOk()) setMode(bhand_::PROTECTIVE_STOP);
  else
  {
    // arma::vec j_vel_cmd = (getJointPosition() - getActualPosition()) / getCtrlCycle();
    arma::vec q_actual = getActualPosition();
    int i = (int)jn;
    double j_vel_cmd = (j_pos(joint_ind) - j_pos_prev(joint_ind)) / getCtrlCycle() + k_click*(j_pos(joint_ind)-q_actual(i));
    hw_i.setJointVelocity(j_vel_cmd, i);
  }
}

void Bh282Robot::setJointVelocity(double vel, bhand_::JointName jn)
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  arma::vec j_vel = arma::vec().zeros(fingers[chain_ind]->getNumJoints());
  j_vel(joint_ind) = vel;
  fingers[chain_ind]->setJointVelocityHelper(j_vel);
  if (!fingers[chain_ind]->isOk()) setMode(bhand_::PROTECTIVE_STOP);
  else
  {
    // arma::vec j_vel_cmd = (getJointPosition() - getActualPosition()) / getCtrlCycle();
    arma::vec j_pos = fingers[chain_ind]->getJointsPosition();
    arma::vec q_actual = getActualPosition();
    int i = (int)jn;
    double j_vel_cmd = j_vel(joint_ind) + k_click*(j_pos(joint_ind)-q_actual(i));
    hw_i.setJointVelocity(j_vel_cmd, i);
  }
}

arma::vec Bh282Robot::getActualPosition()
{
  arma::vec actual_joint_pos(4);
  // get the actual position from the robot
  for (int i=0;i<4;i++) actual_joint_pos(i) = hw_i.getJointPosition(i);

  return actual_joint_pos;
}

void Bh282Robot::checkJointPosDeviationError()
{
  arma::vec actual_joint_pos = getActualPosition();
  arma::vec j_pos = getJointPosition();
  if (arma::norm(j_pos-actual_joint_pos) > 5)
  {
    setMode(bhand_::PROTECTIVE_STOP);
    bhand_::print_err_msg("[Bh282Robot ERROR]: Joint position deviation limit exceeded!\n");
  }
}

void Bh282Robot::initFreedrive()
{
  q_a = arma::vec().zeros(4);
  dq_a = arma::vec().zeros(4);
  ddq_a = arma::vec().zeros(4);
  p_a = 10;
  t_s = 1.0;
}

void Bh282Robot::updateFreedrive()
{
  double d_a = 0.3;
  double k_a = 0.0;
  t_s = 1.5;

  arma::vec torq(4);
  for (int i=0;i<4;i++) torq(i) = -getJointTorque((bhand_::JointName)i);

  // arma::vec dddq_a = -3*p_a*ddq_a -3*std::pow(p_a,2)*dq_a -std::pow(p_a,3)*q_a + t_s*torq;
  ddq_a = -d_a*dq_a -k_a*q_a + t_s*torq;

  std::cout << "torq = " << torq.t() << "\n";
  // std::cout << "q_a = " << q_a.t() << "\n";
  std::cout << "dq_a = " << dq_a.t() << "\n";
  // std::cout << "ddq_a = " << ddq_a.t() << "\n";

  for (int i=0;i<4;i++) setJointVelocity(dq_a(i), (bhand_::JointName)i);
  //setJointVelocity(dq_a, {bhand_::SPREAD, bhand_::FING1, bhand_::FING2, bhand_::FING3});

  double Ts = getCtrlCycle();
  q_a = q_a + dq_a*Ts;
  dq_a = dq_a + ddq_a*Ts;
  // ddq_a = ddq_a + dddq_a*Ts;
}

double Bh282Robot::getJointTorque(bhand_::JointName jn)
{
  return hw_i.getJointTorque((int)jn);
}

}; // namespace bhand_

}; // namespace as64_
