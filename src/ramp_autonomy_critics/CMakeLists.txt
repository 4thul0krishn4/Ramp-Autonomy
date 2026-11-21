cmake_minimum_required(VERSION 3.5)
project(ramp_autonomy_critics)

# Must match nav2_mppi_controller's own build so xtensor/xsimd types have the same layout across the .so boundary.
add_definitions(-DXTENSOR_ENABLE_XSIMD -DXTENSOR_USE_XSIMD)

find_package(ament_cmake REQUIRED)
find_package(xsimd REQUIRED)
find_package(xtensor REQUIRED)
foreach(pkg rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_costmap_2d nav2_common nav2_mppi_controller nav2_util tf2 geometry_msgs nav_msgs)
  find_package(${pkg} REQUIRED)
endforeach()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()
if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()
if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()
add_compile_options(-O2 -Wall)

add_library(ramp_mppi_critics SHARED
  src/ramp_cross_track_critic.cpp
  src/path_probe_critic.cpp)
target_include_directories(ramp_mppi_critics PUBLIC include)
ament_target_dependencies(ramp_mppi_critics
  rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_costmap_2d nav2_util nav2_mppi_controller tf2 geometry_msgs nav_msgs)
target_link_libraries(ramp_mppi_critics xtensor)

pluginlib_export_plugin_description_file(nav2_mppi_controller ramp_critics.xml)

install(TARGETS ramp_mppi_critics LIBRARY DESTINATION lib)
install(FILES ramp_critics.xml DESTINATION share/${PROJECT_NAME})
ament_package()
