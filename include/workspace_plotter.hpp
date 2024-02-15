#pragma once

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

namespace wspltr
{
KDL::Chain chainFromYAML(char* yaml_filename);
KDL::JntArray homePositionFromYAML(char* yaml_filename);

std::map<std::string, std::pair<double, double>> jointLimitsFromYAML(
  char* yaml_filename);

KDL::Frame getEndEffectorPose(const KDL::Chain& arm_chain,
                              const KDL::JntArray& joint_positions);
}
