cmake_minimum_required(VERSION 3.5)

project(phase_manager LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

# ============================================================
# Dependencies
# ============================================================

find_package(Eigen3 3.3 REQUIRED)

# ROS is optional — only required for the ROS server component
find_package(catkin QUIET COMPONENTS
    roscpp
    message_generation
)

# ============================================================
# Core library (no ROS)
# ============================================================

set(LIBRARY_TARGET_NAME phase_manager)

set(phase_manager_SRC
    src/phase_manager.cpp
    src/timeline.cpp
    src/phase.cpp
)

add_library(${LIBRARY_TARGET_NAME} SHARED
    ${phase_manager_SRC}
)

target_include_directories(${LIBRARY_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include/${LIBRARY_TARGET_NAME}>
)

target_link_libraries(${LIBRARY_TARGET_NAME}
    PUBLIC
        Eigen3::Eigen
)

# ============================================================
# ROS server component (optional, only built when catkin found)
# ============================================================

if(catkin_FOUND)

    add_message_files(
        FILES
        Timeline.msg
        TimelineArray.msg
    )

    generate_messages(
        DEPENDENCIES
        std_msgs
    )

    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${LIBRARY_TARGET_NAME} phase_manager_ros
    )

    set(LIBRARY_TARGET_NAME_ROS phase_manager_ros)

    add_library(${LIBRARY_TARGET_NAME_ROS} SHARED
        src/ros_server_class.cpp
    )

    target_include_directories(${LIBRARY_TARGET_NAME_ROS}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${catkin_INCLUDE_DIRS}
    )

    target_link_libraries(${LIBRARY_TARGET_NAME_ROS}
        PUBLIC
            ${LIBRARY_TARGET_NAME}
            ${catkin_LIBRARIES}
    )

    # Propagate the ROS-enabled flag to anything that links phase_manager_ros
    target_compile_definitions(${LIBRARY_TARGET_NAME_ROS}
        PUBLIC PHASE_MANAGER_ROS_ENABLED
    )

    install(TARGETS ${LIBRARY_TARGET_NAME_ROS}
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
    )

    message(STATUS "[phase_manager] ROS components enabled (catkin found)")

else()

    message(STATUS "[phase_manager] ROS components disabled (catkin NOT found)")

endif()

# ============================================================
# Install core library
# ============================================================

install(DIRECTORY include/${LIBRARY_TARGET_NAME}
    DESTINATION include/${LIBRARY_TARGET_NAME}
)

install(
    TARGETS ${LIBRARY_TARGET_NAME}
    EXPORT ${LIBRARY_TARGET_NAME}Targets
    DESTINATION lib
)

include(cmake/ExportProject.cmake)
export_project()

# ============================================================
# Subdirectories
# ============================================================

add_subdirectory(bindings/python/)
add_subdirectory(tests/)
