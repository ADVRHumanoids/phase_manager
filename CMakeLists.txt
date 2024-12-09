cmake_minimum_required(VERSION 3.8)

project(phase_manager)  #LANGUAGES CXX)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
    include/
    src/
    ${EIGEN3_INCLUDE_DIR}
)

set(LIBRARY_TARGET_NAME phase_manager) # set library name

set(phase_manager_SRC
    src/phase_manager.cpp
    src/timeline.cpp
    src/phase.cpp

)

set(phase_manager_ros_SRC
    src/ros_server_class.cpp
)

set(msg_files
    "msg/Timeline.msg"
    "msg/TimelineArray.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME}_msgs
    ${msg_files}
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}_msgs" "rosidl_typesupport_cpp")

add_library(${LIBRARY_TARGET_NAME} SHARED
  ${phase_manager_SRC}
  ${phase_manager_ros_SRC}
)

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${LIBRARY_TARGET_NAME}
    PUBLIC
    rclcpp::rclcpp
    "${cpp_typesupport_target}"
)

target_include_directories(${LIBRARY_TARGET_NAME}
    INTERFACE
    ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS  ${LIBRARY_TARGET_NAME}
        EXPORT   ${LIBRARY_TARGET_NAME}
        LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

#include(InstallBasicPackageFiles)
#install_basic_package_files(${LIBRARY_TARGET_NAME}
#                            VERSION ${${PROJECT_NAME}_VERSION}
#                            COMPATIBILITY AnyNewerVersion
#                            EXPORT ${LIBRARY_TARGET_NAME}
#                            VARS_PREFIX ${LIBRARY_TARGET_NAME}
#                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)

add_subdirectory(bindings/python/)
add_subdirectory(tests/)

ament_export_libraries(${LIBRARY_TARGET_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()