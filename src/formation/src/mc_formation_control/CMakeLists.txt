
find_package(control_toolbox REQUIRED)

add_executable(mc_formation_control_node
    mc_formation_control.cpp
)

ament_target_dependencies(mc_formation_control_node 
        rclcpp px4_msgs px4_ros_com form_msgs 
        control_toolbox Eigen3)
install(TARGETS mc_formation_control_node DESTINATION lib/${PROJECT_NAME})