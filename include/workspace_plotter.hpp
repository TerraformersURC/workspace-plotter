#pragma once

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace wspltr
{
KDL::Chain parseYAML(char *yaml_filename);

KDL::Frame getCartesianPose(KDL::Chain &arm_chain, KDL::JntArray &joint_values);
}
