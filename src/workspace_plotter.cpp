#include <iostream>

#include "workspace_plotter.hpp"

KDL::Chain wspltr::chainFromYAML(char *yaml_filename)
{
  KDL::Chain parsed_chain;

  try {
    YAML::Node arm_node {YAML::LoadFile(yaml_filename)};

    YAML::Node dh_node {arm_node["dh_values"]};

    for (int link_idx {1}; link_idx <= dh_node.size(); link_idx++) {
      std::string link_name {"link" + std::to_string(link_idx)};
      YAML::Node link_node {dh_node[link_name]};

      auto link_frame {KDL::Frame::DH(link_node["r"].as<double>(),
                                      link_node["alpha"].as<double>(),
                                      link_node["d"].as<double>(),
                                      link_node["theta"].as<double>())};

      auto joint_name {"joint" + std::to_string(link_idx)};
      auto joint_z_axis {KDL::Joint(joint_name, KDL::Joint::RotZ)};

      auto link {KDL::Segment(link_name, joint_z_axis, link_frame)};

      parsed_chain.addSegment(link);
    }
  }

  catch(const YAML::BadFile& e) {
    std::cerr << "Parsing the DH parameters failed" << std::endl;
    std::cerr << e.msg << std::endl;
  }

  catch(const YAML::ParserException& e) {
    std::cerr << "Parsing the DH parameters failed" << std::endl;
    std::cerr << e.msg << std::endl;
  }

  std::cout << "Parsed the DH parameters" << std::endl;
  return parsed_chain;
}

KDL::JntArray wspltr::homePositionFromYAML(char *yaml_filename)
{
  KDL::JntArray parsed_position;

  try {
    YAML::Node arm_node {YAML::LoadFile(yaml_filename)};

    std::vector<double> joint_positions {
                           arm_node["home_position"].as<std::vector<double>>()};

    parsed_position.resize(joint_positions.size());

    int link_index {0};
    for (auto joint_position: joint_positions) {
      parsed_position(link_index) = joint_position;

      link_index++;
    }
  }

  catch(const YAML::BadFile& e) {
    std::cerr << "Parsing the arm home position failed" << std::endl;
    std::cerr << e.msg << std::endl;
  }

  catch(const YAML::ParserException& e) {
    std::cerr << "Parsing the arm home position failed" << std::endl;
    std::cerr << e.msg << std::endl;
  }

  std::cout << "Parsed the arm home position" << std::endl;
  return parsed_position;
}

std::map<std::string, std::pair<double, double>> wspltr::jointLimitsFromYAML(
  char* yaml_filename)
{
  std::map<std::string, std::pair<double, double>> joints_limits {};

  try {
    YAML::Node arm_node {YAML::LoadFile(yaml_filename)};
    auto joint_limits_node {arm_node["joint_limits"]};

    for (int joint_idx {1}; joint_idx <= joint_limits_node.size();
                                                                  joint_idx++) {
      std::string joint_name {"joint" + std::to_string(joint_idx)};
      YAML::Node joint_node {joint_limits_node[joint_name]};

      std::pair<double, double> joint_limits {-6.2831, 6.2831};

      joint_limits.first = joint_node["min"].as<double>();
      joint_limits.second = joint_node["max"].as<double>();

      joints_limits[joint_name] = joint_limits;
    }
  }

  catch(const YAML::BadFile& e) {
    std::cerr << "Parsing the joint limits failed" << std::endl;
    std::cerr << e.msg << std::endl;
  }

  catch(const YAML::ParserException& e) {
    std::cerr << "Parsing the joint limits failed" << std::endl;
    std::cerr << e.msg << std::endl;
  }

  std::cout << "Parsed the joint limits" << std::endl;
  return joints_limits;
}

std::vector<KDL::Frame> wspltr::getLinkFrames(const KDL::Chain& arm_chain,
                                           const KDL::JntArray& joint_positions)
{
  KDL::ChainFkSolverPos_recursive forward_solver(arm_chain);
  std::vector<KDL::Frame> link_frames {};

  for (int seg_num {0}; seg_num <= arm_chain.getNrOfSegments(); seg_num++) {
    KDL::Frame link_frame;
  int fk_status {0};

    fk_status = forward_solver.JntToCart(joint_positions, link_frame, seg_num);

    if (fk_status < 0) {
      std::cerr << "FK at segment " << seg_num << " failed!" << std::endl;
      return link_frames;
    }

    link_frames.push_back(link_frame);
  }

  return link_frames;
}

KDL::Frame wspltr::getEndEffectorPose(const KDL::Chain& arm_chain,
                                      const KDL::JntArray& joint_positions)
{
  return wspltr::getLinkFrames(arm_chain, joint_positions).back();
}
}
