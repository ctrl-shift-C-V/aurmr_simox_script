project(MujocoMeshReader)

find_package(Boost ${Simox_BOOST_VERSION} EXACT COMPONENTS system filesystem REQUIRED)


if (Boost_FOUND)

    set(SOURCES
        MujocoMeshReader.cpp
    )

    set(HEADERS
    )

    set(LIBS
        VirtualRobot
    )


    add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
    set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
    set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

    target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})


    #######################################################################################
    ############################ Setup for installation ###################################
    #######################################################################################

    install(TARGETS ${PROJECT_NAME}
        # IMPORTANT: Add the library to the "export-set"
        EXPORT SimoxTargets
        RUNTIME DESTINATION bin COMPONENT bin
        COMPONENT dev)

    message( STATUS " ** Simox application ${PROJECT_NAME} will be placed into " ${Simox_BIN_DIR})
    message( STATUS " ** Simox application ${PROJECT_NAME} will be installed into " bin)

endif()
