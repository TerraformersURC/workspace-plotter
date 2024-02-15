#include "workspace_plotter.hpp"

int main(int argc, char *argv[])
{
  if (!argv[1]) {
    std::cerr << "Missing location to the YAML file, exiting..." << std::endl;
    return 1;
  }

  KDL::Chain parsed_chain {wspltr::chainFromYAML(argv[1])};

  KDL::JntArray initial_joint_angles {wspltr::homePositionFromYAML(argv[1])};

  KDL::Frame initial_pose {wspltr::getEndEffectorPose(parsed_chain,
                                                      initial_joint_angles)};

  KDL::Frame target_pose {initial_pose};
  target_pose.p[0] += 0.1;
  target_pose.p[1] -= 0.1;
  target_pose.p[2] += 0.1;

  auto final_joint_angles {wspltr::getJointPositions(parsed_chain,
                                                     initial_joint_angles,
                                                     target_pose)};

  auto joints_limits {wspltr::jointLimitsFromYAML(argv[1])};

  wspltr::plotWorkspace(parsed_chain, initial_joint_angles, joints_limits);
  // wspltr::showArm(parsed_chain, initial_joint_angles, final_joint_angles);

  return 0;
}
