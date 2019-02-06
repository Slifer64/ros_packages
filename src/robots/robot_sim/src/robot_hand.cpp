#include <robot_sim/robot_hand.h>

namespace as64_
{

namespace robot_sim_
{

RobotHand::RobotHand()
{

}

RobotHand::RobotHand(urdf::Model &urdf_model, const std::vector<std::string> &base_link, const std::vector<std::string> &tool_link,
  double ctrl_cycle, bool check_limits, bool check_singularity, const std::vector<std::string> &wrench_topic)
{
    this->urdf_model = urdf_model;
    this->base_link = base_link;
    this->tool_link = tool_link;
    this->ctrl_cycle = ctrl_cycle;
    this->check_limits = check_limits;
    this->check_singularity = check_singularity;
    this->wrench_topic = wrench_topic;

    init();
}

void RobotHand::init()
{
  N_fingers = tool_link.size();
  fingers.resize(N_fingers);

  for (int i=0; i<N_fingers; i++)
    fingers[i].reset(new RobotChain(urdf_model, base_link[i], tool_link[i], ctrl_cycle, check_limits, check_singularity, wrench_topic[i]));

  setMode(robot_sim_::IDLE);

  // subscribe to topics
  pub_topic = "/robot_sim_joint_states";
  jState_pub = node.advertise<sensor_msgs::JointState>(pub_topic, 1);

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
  for (int i=0;i<N_fingers;i++) fingers[i]->updateState();

  sensor_msgs::JointState joint_state_msg;
  for (int i=0;i<N_fingers;i++) fingers[i]->addJointState(joint_state_msg);
  jState_pub.publish(joint_state_msg);

  //for (int i=0;i<N_fingers;i++) fingers[i]->publishState();

  // wait for timecycle to complete
  unsigned long elaps_time = timer.elapsedNanoSec();
  if (elaps_time < update_time)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(update_time-elaps_time));
  }
  timer.start(); // restart time cycle
}

void RobotHand::setMode(const robot_sim_::Mode &m)
{
  for (int i=0;i<N_fingers;i++)
  {
    fingers[i]->setMode(m);
    if (fingers[i]->getMode() != m) // error occured
    {
      // set all fingers to PROTECTIVE_STOP and return
      for (int j=0; j<N_fingers; j++) fingers[i]->setMode(robot_sim_::PROTECTIVE_STOP);
      mode = robot_sim_::PROTECTIVE_STOP;
      return;
    }
  }
  mode = m;
}

}; // namespace robot_sim_

}; // namespace as64_
