cmake_minimum_required(VERSION 3.8)
project(ramp_autonomy_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config sdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
