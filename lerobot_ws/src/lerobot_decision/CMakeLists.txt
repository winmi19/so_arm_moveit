cmake_minimum_required(VERSION 3.8)
project(lerobot_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  lerobot_decision
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
)

ament_package()
