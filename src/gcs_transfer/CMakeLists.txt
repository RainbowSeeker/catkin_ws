cmake_minimum_required(VERSION 3.8)
project(gcs_transfer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(form_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(MAVLINK_GIT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
set(MAVLINK_DIALECT "gcs_transfer")

# generate the dialect
add_custom_command(
    OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h
    COMMAND
    ${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
    --lang C --wire-protocol 2.0
        #--no-validate
        #--strict-units
    --output ${MAVLINK_LIBRARY_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/message_definitions/${MAVLINK_DIALECT}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${MAVLINK_DIALECT}.log
    DEPENDS
    ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
    ${CMAKE_CURRENT_LIST_DIR}/message_definitions/${MAVLINK_DIALECT}.xml


    COMMENT "Generating Mavlink ${MAVLINK_DIALECT}: ${CMAKE_CURRENT_LIST_DIR}/message_definitions/${MAVLINK_DIALECT}.xml"
)
add_custom_target(mavlink_c_generate DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h PROPERTIES GENERATED true)

# mavlink header only library
add_library(mavlink_c INTERFACE)
target_compile_options(mavlink_c INTERFACE -Wno-address-of-packed-member -Wno-cast-align)
target_sources(mavlink_c
    INTERFACE
    ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h
)
target_include_directories(mavlink_c
    INTERFACE
    ${MAVLINK_LIBRARY_DIR}
    ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}
)

add_executable(gcs_transfer_node
  src/gcs_transfer.cpp
)
target_link_libraries(gcs_transfer_node mavlink_c Boost::system)
ament_target_dependencies(gcs_transfer_node rclcpp form_msgs)
target_include_directories(gcs_transfer_node
    INTERFACE
    ${MAVLINK_LIBRARY_DIR}
    ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}
    ${Boost_INCLUDE_DIR}
)
target_compile_options(gcs_transfer_node PRIVATE -Wno-pedantic)
install(TARGETS
  gcs_transfer_node
  DESTINATION lib/${PROJECT_NAME}
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
