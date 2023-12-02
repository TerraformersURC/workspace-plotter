#include <iostream>

#include "workspace_plotter.hpp"

KDL::Chain wspltr::parseYAML(char *yaml_filename)
{
  KDL::Chain parsed_chain;

  try {
    YAML::Node links_yaml {YAML::LoadFile(yaml_filename)};

    int link_index {1};

    for (auto link_map: links_yaml) {
      std::string link_name {"link" + std::to_string(link_index)};
      YAML::Node link_node {link_map[link_name]};

      auto link_frame {KDL::Frame::DH(link_node["r"].as<double>(),
                                      link_node["alpha"].as<double>(),
                                      link_node["d"].as<double>(),
                                      link_node["theta"].as<double>())};

      // read_parameters.a_length.push_back(link["a_length"].as<double>());
      // read_parameters.d_length.push_back(link["d_lautoength"].as<double>());
      // read_parameters.alpha.push_back(link["alpha"].as<double>());
      // read_parameters.offset.push_back(link["offset"].as<double>());
      // read_parameters.min_limit.push_back(link["min_limit"].as<double>());
      // read_parameters.max_limit.push_back(link["max_limit"].as<double>());

      auto joint_z_axis {KDL::Joint(KDL::Joint::RotZ)};
      auto link {KDL::Segment(joint_z_axis, link_frame)};

      parsed_chain.addSegment(link);

      link_index++;
    }
  }

  catch(const YAML::BadFile& e) {
    std::cerr << e.msg << std::endl;
  }

  catch(const YAML::ParserException& e) {
    std::cerr << e.msg << std::endl;
  }

  return parsed_chain;
}

KDL::Frame wspltr::getCartesianPose(const KDL::Chain &arm_chain, KDL::JntArray &joint_values)
{
  KDL::ChainFkSolverPos_recursive forward_solver(arm_chain);

  KDL::Frame cartesian_pose;
  forward_solver.JntToCart(joint_values, cartesian_pose, arm_chain.getNrOfSegments());

  return cartesian_pose;
}
