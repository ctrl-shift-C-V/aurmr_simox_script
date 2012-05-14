
IF (NOT SIMOX_CONFIGURED)

	# defines SIMOX_CONFIGURED variable which indicates that this config file has already been included
	SET(SIMOX_CONFIGURED TRUE)
		
	# Set up for debug build
	IF(NOT CMAKE_BUILD_TYPE)
	  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
	      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	      FORCE)
	ENDIF(NOT CMAKE_BUILD_TYPE)
	
	
	############################# SETUP PATHS #############################
	SET(SIMOX_BUILD_DIRECTORY ${CMAKE_BINARY_DIR})
	SET(BIN_DIR bin)
	SET(LIB_DIR lib)
	
	SET(SIMOX_LIB_DIR ${SIMOX_BUILD_DIRECTORY}/${LIB_DIR})
	SET(SIMOX_BIN_DIR ${SIMOX_BUILD_DIRECTORY}/${BIN_DIR})
	
	MESSAGE (STATUS "** SIMOX LIB DIR: ${SIMOX_LIB_DIR}")
	MESSAGE (STATUS "** SIMOX BIN DIR: ${SIMOX_BIN_DIR}")
	
	SET(SIMOX_INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/${LIB_DIR})
	SET(SIMOX_INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/${BIN_DIR})
	SET(SIMOX_INSTALL_HEADER_DIR ${CMAKE_INSTALL_PREFIX}/include)
	
	
	## set library settings
	
	    ## SETUP VIRTUAL ROBOT 
    	SET(VIRTUAL_ROBOT_BUILD_DIRECTORY ${SIMOX_BUILD_DIRECTORY})
    	SET(VIRTUAL_ROBOT_INSTALL_LIB_DIR ${SIMOX_INSTALL_LIB_DIR})
        SET(VIRTUAL_ROBOT_INSTALL_BIN_DIR ${SIMOX_INSTALL_BIN_DIR})
        SET(VIRTUAL_ROBOT_INSTALL_HEADER_DIR ${SIMOX_INSTALL_HEADER_DIR})
        include (VirtualRobot/config.cmake)
       
        
        ## SETUP SABA
        # nothing to do yet

        ## SETUP GRASP STUDIO
        # nothing to do yet

     
ENDIF(NOT SIMOX_CONFIGURED)