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

KDL::Frame wspltr::getCartesianPose(KDL::Chain &arm_chain, KDL::JntArray &joint_values)
{
  KDL::ChainFkSolverPos_recursive forward_solver(arm_chain);

  KDL::Frame cartesian_pose;

  int fk_status {0};

  fk_status = forward_solver.JntToCart(joint_values, cartesian_pose, arm_chain.getNrOfSegments());

  (fk_status >= 0) ?
    std::cout << "FK success" << std::endl
    : std::cout << "FK fail" << std::endl;

  return cartesian_pose;
}
