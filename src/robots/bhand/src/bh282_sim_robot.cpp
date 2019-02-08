#include <bhand/bh282_sim_robot.h>

namespace as64_
{

namespace bhand_
{

Bh282SimRobot::Bh282SimRobot(urdf::Model &urdf_model, const std::vector<std::string> &base_link,
                      const std::vector<std::string> &tool_link, double ctrl_cycle)
                      :RobotHand(urdf_model, base_link, tool_link, ctrl_cycle)
{}

Bh282SimRobot::Bh282SimRobot(const std::string &robot_desc_param, const std::vector<std::string> &base_link,
                      const std::vector<std::string> &tool_link, double ctrl_cycle)
                      :RobotHand(urdf_model, base_link, tool_link, ctrl_cycle)
{}

Bh282SimRobot::Bh282SimRobot()
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

  init();
}



}; // namespace bhand_

}; // namespace as64_
