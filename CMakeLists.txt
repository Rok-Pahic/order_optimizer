cmake_minimum_required(VERSION 3.8)
project(order_optimizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OrderDemand.msg"
 )


set( dependencies
  rclcpp 
  geometry_msgs
  visualization_msgs
  yaml-cpp
  tf2

 )




# INSTALL C++ STUFF
add_executable(order_optimizer_node src/order_optimizer_node.cpp)

ament_target_dependencies(order_optimizer_node ${dependencies} )

# target_include_directories(order_optimizer_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# Link against the generated interfaces
rosidl_target_interfaces(order_optimizer_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(order_optimizer_node  ${libyaml_vendor_LIBRARIES} ${YAML_CPP_LIBRARIES} )

install(TARGETS order_optimizer_node
  DESTINATION lib/${PROJECT_NAME})


############ Install launch and config files #############

#Install launch 
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)
#Install config 
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}
)

#Install test data
install(DIRECTORY test_data DESTINATION share/${PROJECT_NAME}
)

#Install rviz data
install(DIRECTORY rviz2 DESTINATION share/${PROJECT_NAME}
)



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
