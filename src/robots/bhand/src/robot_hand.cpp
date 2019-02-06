#include <bhand/robot_hand.h>

namespace as64_
{

namespace bhand_
{

RobotHand::RobotHand(urdf::Model &urdf_model, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link,
  double ctrl_cycle, bool check_limits, bool check_singularity, const std::string &pub_state_topic, const std::vector<std::string> &wrench_topic)
{
    this->urdf_model = urdf_model;
    this->base_link = base_link;
    this->tool_link = tool_link;
    this->ctrl_cycle = ctrl_cycle;
    this->check_limits = check_limits;
    this->check_singularity = check_singularity;
    this->pub_state_topic = pub_state_topic;
    this->wrench_topic = wrench_topic;
}

void RobotHand::init()
{
  N_fingers = tool_link.size();
  fingers.resize(N_fingers);

  // subscribe to topics
  jState_pub = node.advertise<sensor_msgs::JointState>(pub_state_topic, 1);

  for (int i=0; i<N_fingers; i++)
    fingers[i].reset(new RobotChain(urdf_model, base_link[i], tool_link[i], ctrl_cycle, check_limits, check_singularity, wrench_topic[i]));

  setMode(bhand_::IDLE);

  update_time = (int)(getCtrlCycle()*1e9);
  timer.start();
}

RobotHand::~RobotHand() {}

bool RobotHand::isOk() const
{
  bool is_ok = true;

  for (int i=0; i<N_fingers; i++) is_ok &= fingers[i]->isOk();

  return is_ok;
}

void RobotHand::enable()
{
  for (int i=0; i<N_fingers; i++) fingers[i]->enable();
}

std::string RobotHand::getErrMsg() const
{
  std::string err_msg;

  for (int i=0;i<N_fingers;i++)
  {
    std::string msg = fingers[i]->getErrMsg();
    if (msg.compare("")) err_msg += msg + "\n";
  }

  return err_msg;
}

void RobotHand::update()
{
  updateState(); // update state
  if (getMode() != bhand_::FREEDRIVE) publishState(); // publish joint state
  waitNextCycle(); // wait for timecycle to complete
}

void RobotHand::updateState()
{
  for (int i=0;i<N_fingers;i++) fingers[i]->updateState();
}

void RobotHand::publishState()
{
  if (getMode() == bhand_::FREEDRIVE) return;

  sensor_msgs::JointState joint_state_msg;
  for (int i=0;i<N_fingers;i++) fingers[i]->addJointState(joint_state_msg);
  jState_pub.publish(joint_state_msg);
}

void RobotHand::waitNextCycle()
{
  unsigned long elaps_time = timer.elapsedNanoSec();
  if (elaps_time < update_time)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(update_time-elaps_time));
  }
  timer.start(); // restart time cycle
}

void RobotHand::setMode(const bhand_::Mode &m)
{
  for (int i=0;i<N_fingers;i++)
  {
    fingers[i]->setMode(m);
    if (fingers[i]->getMode() != m) // error occured
    {
      // set all fingers to PROTECTIVE_STOP and return
      for (int j=0; j<N_fingers; j++) fingers[i]->setMode(bhand_::PROTECTIVE_STOP);
      mode = bhand_::PROTECTIVE_STOP;
      return;
    }
  }
  mode = m;
}

void RobotHand::getChainFingerInd(bhand_::JointName jn, int &chain_ind, int &joint_ind) const
{
  if (jn == bhand_::SPREAD)
  {
    chain_ind = 0; // spread joint is in the first chain
    joint_ind = 0; // and is the first joint of that chain
  }
  else if (jn == bhand_::FING1)
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

void RobotHand::setJointPosition(double pos, bhand_::JointName jn)
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  arma::vec j_pos = fingers[chain_ind]->getJointsPosition();
  j_pos(joint_ind) = pos;
  fingers[chain_ind]->setJointPosition(j_pos);
  if (!fingers[chain_ind]->isOk()) setMode(bhand_::PROTECTIVE_STOP);
}

void RobotHand::setJointPosition(arma::vec j_pos, const std::vector<bhand_::JointName> &jn)
{
  for (int i=0; i<jn.size(); i++)
  {
    setJointPosition(j_pos(i), jn[i]);
    if (!isOk()) return;
  }
}

void RobotHand::setJointVelocity(double vel, bhand_::JointName jn)
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  arma::vec j_vel = arma::vec().zeros(fingers[chain_ind]->getNumJoints());
  j_vel(joint_ind) = vel;
  fingers[chain_ind]->setJointVelocity(j_vel);
  if (!fingers[chain_ind]->isOk()) setMode(bhand_::PROTECTIVE_STOP);
}

void RobotHand::setJointVelocity(arma::vec j_vel, const std::vector<bhand_::JointName> &jn)
{
  for (int i=0; i<jn.size(); i++)
  {
    setJointVelocity(j_vel(i), jn[i]);
    if (!isOk()) break;
  }
}

double RobotHand::getJointPosition(bhand_::JointName jn) const
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  return (fingers[chain_ind]->getJointsPosition())(joint_ind);
}

arma::vec RobotHand::getJointPosition(const std::vector<bhand_::JointName> &jn) const
{
  arma::vec j_pos(jn.size());
  for (int i=0; i<jn.size(); i++) j_pos(i) = getJointPosition(jn[i]);
  return j_pos;
}

double RobotHand::getJointVelocity(bhand_::JointName jn) const
{
  int chain_ind;
  int joint_ind;
  getChainFingerInd(jn, chain_ind, joint_ind);

  return (fingers[chain_ind]->getJointsVelocity())(joint_ind);
}

arma::vec RobotHand::getJointVelocity(const std::vector<bhand_::JointName> &jn) const
{
  arma::vec j_vel(jn.size());
  for (int i=0; i<jn.size(); i++) j_vel(i) = getJointVelocity(jn[i]);
  return j_vel;
}

bool RobotHand::setJointTrajectory(arma::vec j_target, double duration, const std::vector<bhand_::JointName> &jn)
{
  // keep last known robot mode
  bhand_::Mode prev_mode = getMode();

  // initialize position
  arma::vec q0(jn.size());
  for (int i=0; i<q0.size(); i++) q0(i) = getJointPosition(jn[i]);
  arma::vec qref = q0;

  // initalize time
  double t = 0.0;
  double Ts = getCtrlCycle();

  // start conttroller
  setMode(bhand_::Mode::JOINT_POS_CONTROL);
  while (isOk() && getMode()!=bhand_::Mode::IDLE && t<duration)
  {
    // std::cout << "t = " << t  << " sec\n";
    update();
    t += Ts;
    qref = get5thOrder(t, q0, j_target, duration)[0];
    setJointPosition(qref, jn);
    // for (int i=0; i<qref.size() && isOk(); i++) setJointPosition(qref(i),jn[i]);
  }

  bool reached_target = t>=duration;

  if (isOk() && getMode()==bhand_::Mode::JOINT_POS_CONTROL) setMode(prev_mode);

  return reached_target;
}

}; // namespace bhand_

}; // namespace as64_
