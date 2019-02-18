#include <bhand_test/utils.h>

using namespace as64_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_bhand_robot_test");
  ros::NodeHandle nh("~");

  // give some time to other nodes (rviz) to start as well
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // ============================================
  // =========== Parse values  ==================
  // ============================================

  // initialize parser providing the path to the config file

  std::string robot_description_name;
  std::vector<std::string> base_link;
  std::vector<std::vector<std::string>> tool_link;
  std::vector<double> ctrl_cycle;

  std::vector<arma::vec> q1;
  std::vector<arma::vec> q2;
  std::vector<double> time_duration;
  std::vector<bool> use_sim;

  if (!nh.getParam("robot_description_name",robot_description_name)) throw std::ios_base::failure("Failed to read parameter \"robot_description_name\".");

  int k = 0;

  try{
    while (true)
    {
      std::string bl;
      std::vector<std::string> tl;
      double cc;
      std::vector<double> j1;
      std::vector<double> j2;
      double td;
      bool usim;

      std::ostringstream oss;
      oss << "_" << k+1;
      std::string suffix = oss.str();

      if (!nh.getParam("base_link"+suffix,bl)) throw std::ios_base::failure("Failed to read parameter \"base_link" + suffix + "\".");
      if (!nh.getParam("tool_link"+suffix,tl)) throw std::ios_base::failure("Failed to read parameter \"tool_link" + suffix + "\".");
      if (!nh.getParam("ctrl_cycle"+suffix,cc)) throw std::ios_base::failure("Failed to read parameter \"ctrl_cycle" + suffix + "\".");

      if (!nh.getParam("q1"+suffix,j1)) throw std::ios_base::failure("Failed to read parameter \"q1" + suffix + "\".");
      if (!nh.getParam("q2"+suffix,j2)) throw std::ios_base::failure("Failed to read parameter \"q2" + suffix + "\".");
      if (!nh.getParam("time_duration"+suffix,td)) throw std::ios_base::failure("Failed to read parameter \"time_duration" + suffix + "\".");
      if (!nh.getParam("use_sim"+suffix,usim)) throw std::ios_base::failure("Failed to read parameter \"use_sim" + suffix + "\".");

      k++;
      base_link.push_back(bl);
      tool_link.push_back(tl);
      ctrl_cycle.push_back(cc);
      q1.push_back(j1);
      q2.push_back(j2);
      time_duration.push_back(td);
      use_sim.push_back(usim);
    }
  }
  catch(std::exception &e)
  {
    if (k==0) throw e;
  }

  int N_robots = k;

  // =========================================
  // =========== initialize robots ===========
  // =========================================
  std::vector<std::shared_ptr<bhand_::RobotHand>> bhand_robot(N_robots);
  for (int i=0; i<N_robots; i++)
  {
    if (use_sim[i]) bhand_robot[i].reset(new bhand_::Bh282SimRobot(robot_description_name, base_link[i], tool_link[i], ctrl_cycle[i]));
    else bhand_robot[i].reset(new bhand_::Bh282Robot(robot_description_name, base_link[i], tool_link[i], ctrl_cycle[i]));
    bhand_robot[i]->setJointLimitCheck(true);
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
  for (int i=0; i<N_robots; i++) jState_pub.addFun(&bhand_::Bh282SimRobot::addJointState, bhand_robot[i].get());

  jState_pub.start(); // launches joint states publisher thread

  // ========================================================
  // =========== Launch robots execution threads ============
  // ========================================================
  std::vector<std::thread> robot_run_thead(N_robots);
  std::vector<std::shared_ptr<ExecArgs>> args(N_robots);
  for (int i=0; i<N_robots; i++)
  {
    args[i].reset(new ExecArgs(q1[i], q2[i], time_duration[i], bhand_robot[i].get()));
    robot_run_thead[i] = std::thread(robotRun, args[i].get());
  }

  for (int i=0; i<N_robots; i++)
  {
    if (robot_run_thead[i].joinable()) robot_run_thead[i].join();
  }

  jState_pub.stop();

  return 0;
}
