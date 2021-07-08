PROJECT ( SimoxQtUtility )

MESSAGE (STATUS "\n***** CONFIGURING Simox project SimoxQtUtility *****")

#################################### FILES ##################################
SET(SOURCES

)

SET(INCLUDES

)

if (qwt_FOUND)
    list(APPEND INCLUDES
        qwt/SimoxQwtColorMap.h
        qwt/SimoxQwtSpectrogramData.h
    )
    list(APPEND SOURCES
        qwt/SimoxQwtColorMap.cpp
        qwt/SimoxQwtSpectrogramData.cpp
    )
endif()


# Generate subdirectory headers
simox_generate_subdir_headers(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
)


#######################################################################
# Add library
#######################################################################

set(CMAKE_INCLUDE_CURRENT_DIR OFF)
add_library(SimoxQtUtility SHARED
    ${SOURCES}
    ${INCLUDES}
)

target_include_directories(SimoxQtUtility PUBLIC
    $<BUILD_INTERFACE:${Simox_SOURCE_DIR}>
    $<BUILD_INTERFACE:${Simox_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(SimoxQtUtility PUBLIC ${Simox_OS_compile_options})
target_link_libraries(SimoxQtUtility PUBLIC SimoxUtility)


#######################################################################
# Setup dependencies
#######################################################################
####
if ( Qt5_FOUND )
    MESSAGE (STATUS "Found Qt5: " ${Qt5Gui_INCLUDE_DIRS}  ${Qt5OpenGL_INCLUDE_DIRS} )
    target_link_libraries(SimoxQtUtility PUBLIC Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets)
else ( )
    MESSAGE (STATUS "Did not find Qt. Disabling Qt support.")
endif ( )


if (qwt_FOUND)
    target_include_directories(SimoxQtUtility SYSTEM PUBLIC ${qwt_INCLUDE_DIR})
    target_link_libraries(SimoxQtUtility PUBLIC ${qwt_LIBRARIES})
else ()
    MESSAGE (STATUS "Did not find Qwt. Disabling Qwt support.")
endif()


##########################################################################################################
##########################################################################################################
##########################################################################################################
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

SET_TARGET_PROPERTIES(SimoxQtUtility PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
SET_TARGET_PROPERTIES(SimoxQtUtility PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
SET_TARGET_PROPERTIES(SimoxQtUtility PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})


#######################################################################################
############################ Setup for installation ###################################
#######################################################################################

INSTALL(TARGETS SimoxQtUtility
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION bin COMPONENT bin
    LIBRARY DESTINATION lib COMPONENT shlib
    ARCHIVE DESTINATION lib COMPONENT library
    COMPONENT dev
)

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

MESSAGE (STATUS "***** Finished CONFIGURING Simox project SimoxQtUtility *****\n")
