cmake_minimum_required(VERSION 3.8)
project(gz_human_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(gz_human_actor_animation_control SHARED
  src/actor_animation_control_plugin.cpp
)
target_link_libraries(gz_human_actor_animation_control
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)

install(PROGRAMS
  scripts/human_cmd_vel_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  gz_human_actor_animation_control
  DESTINATION lib
)

install(DIRECTORY
  config
  launch
  models
  scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
