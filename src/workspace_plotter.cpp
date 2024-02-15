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

KDL::JntArray wspltr::getJointPositions(const KDL::Chain& arm_chain,
                                  const KDL::JntArray& initial_joint_positions,
                                  const KDL::Frame& target_end_effector_pose,
                                  float time_seconds)
{
  KDL::JntArray joint_positions(arm_chain.getNrOfJoints());

  KDL::ChainFkSolverPos_recursive forward_pos_solver(arm_chain);
  KDL::ChainIkSolverVel_pinv inverse_vel_solver(arm_chain);
  KDL::ChainIkSolverPos_NR inverse_pos_solver(arm_chain, forward_pos_solver,
                                              inverse_vel_solver, 100, 1e-6);
  int ik_status {};

  ik_status = inverse_pos_solver.CartToJnt(initial_joint_positions,
                                           target_end_effector_pose,
                                           joint_positions);

  if (ik_status < 0) {
    std::cerr << "IK solve failed!" << std::endl;
  }

  return joint_positions;
}

matplot::line_handle wspltr::plotArm(const KDL::Chain& arm_chain,
                                     const KDL::JntArray& joint_positions,
                                     const std::string& color)
{
  std::vector<double> frame_x_points {};
  std::vector<double> frame_y_points {};
  std::vector<double> frame_z_points {};

  auto link_frames {wspltr::getLinkFrames(arm_chain, joint_positions)};

  for (const auto& link_frame: link_frames) {
    frame_x_points.push_back(link_frame.p[0]);
    frame_y_points.push_back(link_frame.p[1]);
    frame_z_points.push_back(link_frame.p[2]);
  }

  auto plot {matplot::plot3(frame_x_points, frame_y_points, frame_z_points,
                                                                        "g-o")};
  plot->line_width(3).color(color);

  return plot;
}

void wspltr::showArm(const KDL::Chain& arm_chain,
                     const KDL::JntArray& joint_positions)
{
  auto plot {wspltr::plotArm(arm_chain, joint_positions)};

  matplot::xlim({-0.5, 0.5});
  matplot::ylim({-0.5, 0.5});
  matplot::zlim({0, 1});
  matplot::grid(true);
  matplot::xlabel("X Axis");
  matplot::ylabel("Y Axis");
  matplot::zlabel("Z Axis");

  matplot::show();
}

void wspltr::showArm(const KDL::Chain& arm_chain,
                     const KDL::JntArray& initial_joint_positions,
                     const KDL::JntArray& final_joint_positions)
{
  auto plot1 {wspltr::plotArm(arm_chain, initial_joint_positions, "blue")};

  matplot::hold(matplot::on);

  auto plot2 {wspltr::plotArm(arm_chain, final_joint_positions, "red")};

  matplot::xlim({-0.5, 0.5});
  matplot::ylim({-0.5, 0.5});
  matplot::zlim({0, 1});
  matplot::grid(true);
  matplot::xlabel("X Axis");
  matplot::ylabel("Y Axis");
  matplot::zlabel("Z Axis");

  matplot::hold(matplot::off);

  matplot::show();
}

void wspltr::randomizeJointPositions(int start_joint,
                                     KDL::JntArray& joint_positions,
                                     const std::map<std::string,
                                      std::pair<double, double>>& joints_limits,
                                     std::mt19937& random_number_generator)
{

  for (int joint_idx {start_joint}; joint_idx < joint_positions.rows();
                                                                  joint_idx++) {

    auto joint_limits {joints_limits.at("joint" + std::to_string(joint_idx + 1))};

    std::uniform_real_distribution<double> joint_positions_distribution(
                                       joint_limits.first, joint_limits.second);

    joint_positions(joint_idx) = joint_positions_distribution(
                                                     random_number_generator);
  }
}

// Using Monte Carlo method
void wspltr::plotWorkspace(const KDL::Chain& arm_chain,
                           const KDL::JntArray& initial_joint_angles,
                           const std::map<std::string,
                                   std::pair<double, double>>& joints_limits)
{
  std::vector<double> x_points {};
  std::vector<double> y_points {};
  std::vector<double> z_points {};
  std::vector<double> marker_sizes {};

  KDL::JntArray joint_positions {initial_joint_angles};

  std::random_device random_device;
  std::mt19937 random_position_generator(random_device());

  int iterations_number = 360 * 100;

  for (int iteration {0}; iteration <= iterations_number; iteration++) {

    wspltr::randomizeJointPositions(0, joint_positions, joints_limits,
                                    random_position_generator);

    auto endeff_pose {wspltr::getEndEffectorPose(arm_chain, joint_positions)};

    x_points.push_back(endeff_pose.p[0]);
    y_points.push_back(endeff_pose.p[1]);
    z_points.push_back(endeff_pose.p[2]);

    marker_sizes.push_back(1.0);

    float progress {((float)iteration / iterations_number) * 100};
    std::cout << "Progress: " << progress << " %" << " \r";
    std::cout.flush();
  }

  auto arm_plot {wspltr::plotArm(arm_chain, initial_joint_angles)};

  matplot::hold(matplot::on);

  auto workspace_plot {matplot::scatter3(x_points, y_points, z_points,
                                                                 marker_sizes)};

  matplot::xlim({-0.5, 0.5});
  matplot::ylim({-0.5, 0.5});
  matplot::zlim({-0.5, 0.5});
  matplot::grid(true);
  matplot::xlabel("X Axis");
  matplot::ylabel("Y Axis");
  matplot::zlabel("Z Axis");

  matplot::hold(matplot::off);

  matplot::show();
}
