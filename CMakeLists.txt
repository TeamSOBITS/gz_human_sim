cmake_minimum_required(VERSION 3.8)
project(gz_human_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Let AUTOMOC process .hh headers (HumanControlPanel.hh uses that
# extension, matching this package's existing plugin source convention).
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Boost REQUIRED)

# DualSense/gamepad polling for HumanControlPanel's teleop mode -- SDL2's
# GameController API (not a ROS joy node) so the plugin stays self-contained
# like every other GUI plugin here (gz-transport/Qt only, no rclcpp). Mirrors
# guide_robot's GuiderRobotManager, which this mode is modeled on.
find_package(SDL2 REQUIRED)

add_library(gz_human_actor_animation_control SHARED
  src/server/actor_animation_control_plugin.cpp
)
target_link_libraries(gz_human_actor_animation_control
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)

add_library(gz_human_actor_command SHARED
  src/server/actor_command_plugin.cpp
)
# include/gz_human_sim/: サーバー側と GUI 側が共有する型だけを置く場所
# （CharacterState.hh など）。Gazebo にも Qt にも依存しない。
target_include_directories(gz_human_actor_command PRIVATE include)
target_link_libraries(gz_human_actor_command
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

# lightsfm (third_party/lightsfm) is vendored directly into this package --
# see third_party/lightsfm/README.md -- rather than depended on as a
# separate colcon package, so this builds standalone if the sibling
# lightsfm/sfm_for_gazebo clones this was developed alongside are removed.
add_library(gz_human_sfm_crowd_system SHARED
  src/server/sfm_crowd_system.cpp
)
target_include_directories(gz_human_sfm_crowd_system PRIVATE
  third_party/lightsfm/include
)
target_link_libraries(gz_human_sfm_crowd_system
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  Boost::boost
)

# Collision-aware global path planner (occupancy grid + inflation + A*),
# mirroring the Nav2 layering sobits_navigation_stack uses for the robots --
# see the file header for the layer-by-layer correspondence.
add_library(gz_human_nav_grid_system SHARED
  src/server/nav_grid_system.cpp
)
target_link_libraries(gz_human_nav_grid_system
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

# GUI plugin: spawns/removes/teleoperates gz_human_sim humans from the
# Gazebo sidebar. Kept in its own lib/gz-gui search path, separate from
# the gz-sim System plugins above (see GZ_GUI_PLUGIN_PATH env-hook below).
# The implementation is split across one .cc per responsibility (all the
# same HumanControlPanel class -- see src/gui/HumanControlPanelInternal.hh
# for the tables and constants they share), rather than the single
# ~3700-line file this used to be.
add_library(HumanControlPanel SHARED
  src/gui/HumanControlPanel.cc
  src/gui/HumanControlPanelCamera.cc
  src/gui/HumanControlPanelCollision.cc
  src/gui/HumanControlPanelDualsense.cc
  src/gui/HumanControlPanelInput.cc
  src/gui/HumanControlPanelOverlay.cc
  src/gui/HumanControlPanelRoute.cc
  src/gui/HumanControlPanelSpawn.cc
  src/gui/HumanControlPanelState.cc
  src/gui/HumanControlPanelTeleop.cc
  src/gui/HumanControlPanel.hh
  src/gui/HumanControlPanelInternal.hh
  gui/HumanControlPanel.qrc
)
target_compile_features(HumanControlPanel PUBLIC cxx_std_17)
set_target_properties(HumanControlPanel PROPERTIES
  OUTPUT_NAME HumanControlPanel
  INSTALL_RPATH_USE_LINK_PATH TRUE
  AUTOMOC ON
  AUTORCC ON
)
target_include_directories(HumanControlPanel PRIVATE include)
target_link_libraries(HumanControlPanel
  gz-gui8::gz-gui8
  gz-rendering8::gz-rendering8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  Qt5::Core
  SDL2::SDL2
)

install(PROGRAMS
  scripts/human_cmd_vel_controller.py
  scripts/human_teleop_switcher.py
  scripts/path_template.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  gz_human_actor_animation_control
  gz_human_actor_command
  gz_human_sfm_crowd_system
  gz_human_nav_grid_system
  DESTINATION lib
)

install(TARGETS
  HumanControlPanel
  DESTINATION lib/${PROJECT_NAME}/gz-gui
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  gui
  launch
  # media/: the magic-circle texture HumanControlPanel maps onto its spawn
  # markers. guide_robot ships an identical copy for its own robot spawn
  # marker; regenerate both with
  # `ros2 run guider_multifloor_builder guider_make_spawn_marker`.
  media
  models
  rviz
  scripts
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(env-hooks/gz_human_sim.sh.in)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
