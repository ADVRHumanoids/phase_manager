cmake_minimum_required(VERSION 3.16)

project(phase_manager LANGUAGES C CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    msg/Timeline.msg
    msg/TimelineArray.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

set(LIBRARY_TARGET_NAME phase_manager_core)

add_library(${LIBRARY_TARGET_NAME} SHARED
    src/phase_manager.cpp
    src/timeline.cpp
    src/phase.cpp
)

set_target_properties(${LIBRARY_TARGET_NAME}
    PROPERTIES
        OUTPUT_NAME phase_manager
        EXPORT_NAME phase_manager
)

target_include_directories(${LIBRARY_TARGET_NAME}
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${LIBRARY_TARGET_NAME}
    PUBLIC
        Eigen3::Eigen
)

add_library(phase_manager_ros SHARED
    src/ros_server_class.cpp
)

target_include_directories(phase_manager_ros
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(phase_manager_ros
    PUBLIC
        ${LIBRARY_TARGET_NAME}
        rclcpp::rclcpp
        "${cpp_typesupport_target}"
)

target_compile_definitions(phase_manager_ros
    PUBLIC PHASE_MANAGER_ROS_ENABLED
)

install(
    TARGETS ${LIBRARY_TARGET_NAME} phase_manager_ros
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    EXPORT ${PROJECT_NAME}Targets
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake
)

install(
    FILES
        python/gui.py
        python/ros_client_class.py
        python/timelines_widget.py
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

add_subdirectory(bindings/python)
add_subdirectory(tests)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    Eigen3
    rclcpp
    rosidl_default_runtime
)

ament_package()
