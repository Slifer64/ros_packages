#include <iostream>
#include <cstdlib>
#include <memory>

#include <ros/ros.h>
#include <bhand/bhand_API/BhandHWInterface.h>

using namespace as64_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bh282_bhand");

  BhandHWInterface hw_i;

  hw_i.initialize();
  hw_i.terminate();

  return 0;
}
