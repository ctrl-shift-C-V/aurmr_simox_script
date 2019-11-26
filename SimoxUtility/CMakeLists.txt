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

    math/convert/deg2rad.h
    math/convert/rad2deg.h
)

macro(simox_subdirs result curdir)
    file(GLOB children ${curdir}/*)
    set(${result})
    foreach(child ${children})
        if(IS_DIRECTORY ${child})
            list(APPEND ${result} ${child})
        endif()
    endforeach()
endmacro()

macro(simox_update_file file content) #macro since we want to provide simox_file_up_to_date
    set(simox_file_up_to_date 0)
    if(EXISTS "${file}")
        file(SHA512 "${file}" _simox_file_sha)
        string(SHA512 _simox_cont_sha "${content}")
        if("${_simox_cont_sha}" STREQUAL "${_simox_file_sha}")
            set(simox_file_up_to_date 1)
        endif()
    endif()
    #write file
    if(${simox_file_up_to_date} EQUAL 0)
        file(WRITE "${file}" "${content}")
    endif()
endmacro()

macro(simox_generate_subdir_headers dir prefix_to_remove out_headers)
    string(REPLACE "//" "/" prefix_to_remove_fixed "${prefix_to_remove}/")
    string(REPLACE "${prefix_to_remove_fixed}" "" dir_rel "${dir}")
    _simox_generate_subdir_headers_impl("${dir}" "${prefix_to_remove_fixed}" ${out_headers})
endmacro()

macro(_simox_generate_subdir_headers_impl dir prefix_to_remove out_headers)
    simox_subdirs(subdirs ${dir})
    foreach(subdir ${subdirs})
        #recurse
        _simox_generate_subdir_headers_impl("${subdir}" "${prefix_to_remove}" ${out_headers})

        file(GLOB headers ${subdir}/*.h) 
        list(LENGTH headers n)
        string(REGEX REPLACE ".*/" "" subdir_name "${subdir}")
        if(NOT ${n} EQUAL 0 AND NOT "${subdir_name}" STREQUAL "detail")
            set(subdir_header_abs "${subdir}.h")
            string(REPLACE "${prefix_to_remove}" "" subdir_header "${subdir_header_abs}")
            list(APPEND ${out_headers} ${subdir_header})
            #create file content
            set(content "#pragma once\n\n// This file is generated!\n\n")
            foreach(header ${headers})
                string(REPLACE "${subdir}/" "${subdir_name}/" h "${header}")
                set(content "${content}#include \"${h}\"\n")
            endforeach()
            #check for file change
            simox_update_file("${subdir_header_abs}" "${content}")
        endif()
    endforeach()
endmacro()

simox_generate_subdir_headers(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
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
