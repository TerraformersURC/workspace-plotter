#include "workspace_plotter.hpp"

int main(int argc, char *argv[])
{
  if (!argv[1]) {
    std::cerr << "Missing location to the YAML file, exiting..." << std::endl;
    return 1;
  }

  KDL::Chain parsed_chain {wspltr::parseYAML(argv[1])};

  std::cout << "Number of joints in parsed chain: " <<
                                      parsed_chain.getNrOfJoints() << std::endl;

  std::cout << "Number of links in parsed chain: " <<
                                    parsed_chain.getNrOfSegments() << std::endl;

  std::cout << "Getting cartesian pose for sample joint angles..." << std::endl;

  std::cout << "Joint angles: " << std::endl;

  KDL::JntArray sample_joint_angles {parsed_chain.getNrOfSegments()};
  sample_joint_angles(0) = M_PI_2;
  sample_joint_angles(1) = -M_PI_2;
  sample_joint_angles(2) = M_PI_2;
  sample_joint_angles(3) = -M_PI_2;
  sample_joint_angles(4) = -M_PI_2;
  sample_joint_angles(5) = 0;

  std::cout << sample_joint_angles.data << std::endl;

  auto final_pose {wspltr::getCartesianPose(parsed_chain, sample_joint_angles)};

  std::cout << final_pose << std::endl;

  return 0;
}
