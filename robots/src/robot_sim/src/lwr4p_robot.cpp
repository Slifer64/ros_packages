#include <robot_sim/lwr4p_robot.h>

namespace as64_
{

namespace robot_sim_
{

LWR4pRobot::LWR4pRobot()
{
  if (!node.getParam("/lwr4p_robot/base_frame",base_link_name))
  {
    throw std::runtime_error("Failed to load parameter \"/lwr4p_robot/base_frame\" ...\n");
  }

  if (!node.getParam("/lwr4p_robot/tool_frame",tool_link_name))
  {
    throw std::runtime_error("Failed to load parameter \"/lwr4p_robot/tool_frame\" ...\n");
  }

  if (!node.getParam("/lwr4p_robot/check_limits",check_limits))
  {
    check_limits = false;
  }

  if (!node.getParam("/lwr4p_robot/check_singularity",check_singularity))
  {
    check_singularity = false;
  }

  if (!node.getParam("/lwr4p_robot/ctrl_cycle",ctrl_cycle))
  {
    ctrl_cycle = 0.01;
  }

  if (node.getParam("/lwr4p_robot/wrench_topic",wrench_topic))
  {
    read_wrench_from_topic = true;
  }

  std::string robot_description_name;
  if (!node.getParam("/lwr4p_robot/robot_description_name",robot_description_name))
  {
    throw std::runtime_error("Failed to load parameter \"/lwr4p_robot/robot_description_name\" ...\n");
  }

  //std::string urdf_file_path = ros::package::getPath("robot_sim") + "/urdf/lwr4p_robot.urdf";

  if (!urdf_model.initParam(robot_description_name.c_str()))
  // if (!urdf_model.initFile(urdf_file_path.c_str()))
  {
    throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_description_name + "\"...\n");
  }

  init();
}

}; // namespace robot_sim_

}; // namespace as64_
