#include "workspace_plotter.hpp"

int main(int argc, char *argv[])
{
  if (!argv[1]) {
    std::cerr << "Missing location to the YAML file, exiting..." << std::endl;
    return 1;
  }

  KDL::Chain parsed_chain {wspltr::chainFromYAML(argv[1])};

  // TODO: Implement CLI
  // char user_input {};
  // do {
  //   std::cout << "Workspace plotter: " << std::endl;
  //   std::cout << "1 - Display Arm" << std::endl;
  //   std::cout << "2 - Plot Workspace" << std::endl;
  //   std::cout << "3 - Inverse Kinematics" << std::endl;

  //   std::cout << std::endl << "Option: ";
  //   std::cin >> user_input;

  //   // switch (user_input)

  // } while (user_input != 'e');

  KDL::JntArray initial_joint_angles {wspltr::homePositionFromYAML(argv[1])};

  // KDL::Frame initial_pose {wspltr::getEndEffectorPose(parsed_chain,
  //                                                     initial_joint_angles)};

  // KDL::Frame target_pose {initial_pose};
  // target_pose.p[0] += 0.1;
  // target_pose.p[1] -= 0.1;
  // target_pose.p[2] += 0.1;

  // auto final_joint_angles {wspltr::getJointPositions(parsed_chain,
  //                                                    initial_joint_angles,
  //                                                    target_pose)};

  auto joints_limits {wspltr::jointLimitsFromYAML(argv[1])};

  wspltr::plotWorkspace(parsed_chain, initial_joint_angles, joints_limits);

  KDL::JntArray limit_joint_angles(parsed_chain.getNrOfJoints());
  limit_joint_angles(0) = 3.1415;
  limit_joint_angles(1) = 0.2618;
  limit_joint_angles(2) = 0.5236;
  limit_joint_angles(3) = 3.1415;
  limit_joint_angles(4) = 3.1415;
  wspltr::showArm(parsed_chain, initial_joint_angles, limit_joint_angles);

  return 0;
}
