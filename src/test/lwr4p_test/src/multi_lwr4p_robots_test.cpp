#include <lwr4p_test/utils.h>

using namespace as64_;

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
  std::string confi_file_path = ros::package::getPath("lwr4p_test") + "/config/multi_lwr4p_robots_test.yaml";
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
    // lwr4p_robot[i]->readWrenchFromTopic("/wrench");
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

  // ========================================================
  // =========== Launch robots execution threads ============
  // ========================================================
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
