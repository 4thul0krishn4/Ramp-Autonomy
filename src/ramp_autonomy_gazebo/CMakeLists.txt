cmake_minimum_required(VERSION 3.8)
project(ramp_autonomy_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
