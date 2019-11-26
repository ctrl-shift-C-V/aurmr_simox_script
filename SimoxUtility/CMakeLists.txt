PROJECT ( SimoxUtility )

add_library(SimoxUtility SHARED) # SimoxUtility.h) # sources will be added later

target_include_directories(SimoxUtility PUBLIC
    $<BUILD_INTERFACE:${Simox_SOURCE_DIR}>
    $<BUILD_INTERFACE:${Simox_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(SimoxUtility PUBLIC ${Simox_OS_compile_options})
target_link_libraries(SimoxUtility PUBLIC stdc++fs)
#######################################################################
# Setup dependencies
#######################################################################
#### Eigen
target_include_directories(SimoxUtility SYSTEM PUBLIC ${Eigen3_INCLUDE_DIR})

#### BOOST
target_link_libraries(SimoxUtility PUBLIC
        Boost::boost
        Boost::program_options
        Boost::thread
        Boost::regex
)

# disable boost auto linking
if (Boost_USE_STATIC_LIBS)
    target_compile_definitions(SimoxUtility PUBLIC -DBOOST_ALL_NO_LIB -DBOOST_TEST_MAIN)
else (Boost_USE_STATIC_LIBS)
    # enable dynamic linking for specific boost libraries
    target_compile_definitions(SimoxUtility PUBLIC
        -DBOOST_ALL_NO_LIB
        -DBOOST_PROGRAM_OPTIONS_DYN_LINK
        -DBOOST_FILESYSTEM_DYN_LINK
        -DBOOST_SYSTEM_DYN_LINK
        -DBOOST_THREAD_DYN_LINK)
endif ()

##########################################################################################################
##########################################################################################################
##########################################################################################################

MESSAGE (STATUS "\n***** CONFIGURING Simox project SimoxUtility *****")

#################################### FILES ##################################
SET(SOURCES
    SimoxUtility.cpp
)

SET(INCLUDES
    SimoxUtility.h
    eigen/OrientedBox.h
    eigen/XYConstrainedOrientedBox.h
)

if(BUILD_TESTING)
    # include unit tests
    ADD_SUBDIRECTORY(tests)
endif()

if(Simox_BUILD_EXAMPLES)
    # include examples
    ADD_SUBDIRECTORY(examples)
endif()

# Finished adding UI files
#-----------------------------------------------------------------------------------
set_target_properties(SimoxUtility PROPERTIES SOURCES "${SOURCES};${INCLUDES}")

# against undefined reference to boost::filesystem::detail::copy_file
# source: https://stackoverflow.com/a/3500721
#target_compile_definitions(SimoxUtility PRIVATE -DBOOST_NO_CXX11_SCOPED_ENUMS)


# .DLL path
SET_TARGET_PROPERTIES(SimoxUtility PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(SimoxUtility PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(SimoxUtility PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})




#######################################################################################
############################ Setup for installation ###################################
#######################################################################################

install(TARGETS SimoxUtility
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION bin COMPONENT bin
    LIBRARY DESTINATION lib COMPONENT shlib
    ARCHIVE DESTINATION lib COMPONENT library
    COMPONENT dev)
 
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} DESTINATION "${Simox_INSTALL_HEADER_DIR}"
    COMPONENT Headers
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        PATTERN ".git" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
    )

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION "."
    COMPONENT ApplicationData
        FILES_MATCHING 
        PATTERN "*.xml"
        PATTERN "*.XML"
        PATTERN "*.iv"
        PATTERN "*.vrml"
        PATTERN "*.wrl"
        PATTERN "*.stl"
        PATTERN "*.urdf"
        PATTERN "*.dae"
        PATTERN "*.png"
        PATTERN "*.jpg"
        PATTERN "*.bin"
        PATTERN ".svn" EXCLUDE
        PATTERN ".git" EXCLUDE
        )

MESSAGE (STATUS "***** Finished CONFIGURING Simox project SimoxUtility *****\n")
