PROJECT ( DepthOffscreenRendering )

IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
    set(demo_SRCS DepthOffscreenRendering.cpp main.cpp)
    set(demo_INCS DepthOffscreenRendering.h)
    set(GUI_MOC_HDRS DepthOffscreenRendering.h)
    set(GUI_UIS DepthOffscreenRendering.ui)

    # create the executable
    VirtualRobotQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")

    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

    find_package(OpenGL REQUIRED)
    if(OPENGL_FOUND)
        target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OPENGL_INCLUDE_DIR})
        TARGET_LINK_LIBRARIES(${PROJECT_NAME} PRIVATE ${OPENGL_LIBRARIES})
    endif()

    #######################################################################################
    ############################ Setup for installation ###################################
    #######################################################################################
    install(TARGETS ${PROJECT_NAME}
        # IMPORTANT: Add the library to the "export-set"
        EXPORT SimoxTargets
        RUNTIME DESTINATION bin COMPONENT bin
        COMPONENT dev)

    MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be placed into " ${Simox_BIN_DIR})
    MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be installed into " bin)

ENDIF()
