add_executable(workspace-plotter
  workspace_plotter.cpp
  main.cpp
)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

target_include_directories(workspace-plotter PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(workspace-plotter PUBLIC
  # list of libraries
  yaml-cpp::yaml-cpp
  orocos-kdl
)
