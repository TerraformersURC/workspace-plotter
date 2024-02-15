#pragma once

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <matplot/matplot.h>

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

std::vector<KDL::Frame> getLinkFrames(const KDL::Chain& arm_chain,
                                      const KDL::JntArray& joint_positions);

matplot::line_handle plotArm(const KDL::Chain& arm_chain,
                             const KDL::JntArray& joint_positions,
                             const std::string& color = "blue");

void showArm(const KDL::Chain& arm_chain, const KDL::JntArray& joint_positions);
void showArm(const KDL::Chain& arm_chain,
             const KDL::JntArray& initial_joint_positions,
             const KDL::JntArray& final_joint_positions);
}
