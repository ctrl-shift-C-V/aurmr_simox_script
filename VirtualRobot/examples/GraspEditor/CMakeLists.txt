PROJECT ( GraspEditor )

IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
    MESSAGE (STATUS " ** Adding GraspEditor gui")

    SET(demo_SRCS    GraspEditorWindow.cpp GraspEditor.cpp)
    SET(demo_INCS    GraspEditorWindow.h)
    SET(GUI_MOC_HDRS ${demo_INCS})
    SET(GUI_UIS      GraspEditor.ui)

	# create the executable
	VirtualRobotQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
	SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
	SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

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
