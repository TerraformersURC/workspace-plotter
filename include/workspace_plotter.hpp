#pragma once

#include <iostream>
#include <random>

#include <yaml-cpp/yaml.h>
#include <matplot/matplot.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
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

KDL::JntArray getJointPositions(const KDL::Chain& arm_chain,
                                const KDL::JntArray& initial_joint_positions,
                                const KDL::Frame& target_end_effector_pose,
                                float time_seconds = 1.0);

matplot::line_handle plotArm(const KDL::Chain& arm_chain,
                             const KDL::JntArray& joint_positions,
                             const std::string& color = "blue");

void showArm(const KDL::Chain& arm_chain, const KDL::JntArray& joint_positions);
void showArm(const KDL::Chain& arm_chain,
             const KDL::JntArray& initial_joint_positions,
             const KDL::JntArray& final_joint_positions);

void plotWorkspace(const KDL::Chain& arm_chain,
                   const KDL::JntArray& initial_joint_angles,
                   const std::map<std::string,
                                  std::pair<double, double>>& joints_limits);
void getRandomJointPositions(const std::pair<double, double>& joint_limits,
                             int size,
                             const std::vector<double>& random_joint_positions);

void randomizeJointPositions(int start_joint,
                             KDL::JntArray& joint_positions,
                             const std::map<std::string,
                                std::pair<double, double>>& joints_limits,
                             std::mt19937& random_number_generator);
}
