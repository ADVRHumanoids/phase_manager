find_package(pybind11 QUIET)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    pybind11_add_module(pyphase pyPhase.cpp)
    target_link_libraries(pyphase PUBLIC ${LIBRARY_TARGET_NAME})
    install(TARGETS pyphase
            LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

    pybind11_add_module(pytimeline pyTimeline.cpp)
    target_link_libraries(pytimeline PUBLIC ${LIBRARY_TARGET_NAME})
    install(TARGETS pytimeline
            LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

    pybind11_add_module(pymanager pyPhaseManager.cpp)
    target_link_libraries(pymanager PUBLIC ${LIBRARY_TARGET_NAME})
    install(TARGETS pymanager
            LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

    if(TARGET phase_manager_ros)
        pybind11_add_module(pyrosserver pyRosServer.cpp)
        target_link_libraries(pyrosserver PUBLIC phase_manager_ros)
        install(TARGETS pyrosserver
                LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
        message(STATUS "[phase_manager/python] pyrosserver enabled")
    else()
        message(STATUS "[phase_manager/python] pyrosserver SKIPPED")
    endif()

#    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#    pybind11_add_module(pyphase_manager pyPhaseManager.cpp)
#    target_link_libraries(pyci PRIVATE phase_manager)

#    install(TARGETS pyphase_manager
#            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

#    install(FILES pyci_all.py
#            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

#    install(PROGRAMS interactive_client.py
#            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

else()
    message(STATUS "pybind not found")
endif()
