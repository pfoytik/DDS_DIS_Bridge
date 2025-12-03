cmake_minimum_required(VERSION 3.8)
project(my_cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ShipEntityState.msg"
  DEPENDENCIES std_msgs
)

add_executable(talker_node src/talker_node.cpp)
target_include_directories(talker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(talker_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  talker_node
  "rclcpp"
  "std_msgs"
)

install(TARGETS talker_node
  DESTINATION lib/${PROJECT_NAME})

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
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

# Add open-dis-cpp library
find_package(PkgConfig REQUIRED)
add_subdirectory(../open-dis-cpp ${CMAKE_CURRENT_BINARY_DIR}/open-dis-cpp)

# Ship publisher executable
add_executable(ship_publisher src/ship_publisher.cpp)
target_include_directories(ship_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ship_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ship_publisher rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ship_publisher "${cpp_typesupport_target}")

# DDS to DIS bridge executable  
add_executable(dds_dis_bridge src/dds_dis_bridge.cpp)
target_include_directories(dds_dis_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ../open-dis-cpp/src)
target_compile_features(dds_dis_bridge PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dds_dis_bridge rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dds_dis_bridge "${cpp_typesupport_target2}" OpenDIS6)

# DIS receiver test executable
add_executable(dis_receiver_test src/dis_receiver_test.cpp)
target_include_directories(dis_receiver_test PUBLIC
  ../open-dis-cpp/src)
target_compile_features(dis_receiver_test PUBLIC c_std_99 cxx_std_17)
target_link_libraries(dis_receiver_test OpenDIS6)

install(TARGETS
  publisher
  subscriber
  ship_publisher
  dds_dis_bridge
  dis_receiver_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
