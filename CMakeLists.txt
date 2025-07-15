cmake_minimum_required(VERSION 3.8)
project(basic_5th_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	system 
	thread 
	program_options
)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(3dPathPlanner REQUIRED)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)
find_package(octomath REQUIRED)

include_directories(include)
# find_library(FCL_STATIC_LIB libfcl.a PATHS /usr/local/lib/)
# include_directories(/usr/include/3dPathPlanner/)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(quintic_planner src/planner.cpp src/5th_polynomial.cpp src/pplanner.cpp)
ament_target_dependencies(quintic_planner rclcpp px4_msgs geometry_msgs Eigen3 Boost tf2 std_msgs nav_msgs fcl ocotomap octomath)
target_include_directories(quintic_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(quintic_planner Eigen3::Eigen ${geometry_msgs_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS quintic_planner DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
