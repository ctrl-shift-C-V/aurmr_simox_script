PROJECT ( SimoxUtility )

set(CMAKE_INCLUDE_CURRENT_DIR OFF)
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

target_compile_options(SimoxUtility PUBLIC -include SimoxUtility/eigen/AssertAsException.h)

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

    color/cmaps/colormaps.cpp
    color/cmaps/Named.cpp
    color/Color.cpp
    color/ColorMap.cpp
    color/GlasbeyLUT.cpp
    color/hsv.cpp
    color/interpolation.cpp
    color/json.cpp

    error/SimoxError.cpp

    filesystem/list_directory.cpp

    json/eigen_conversion.cpp
    json/io.cpp
    json/converters.cpp

    math/pose/align_box_orientation.cpp
    math/pose/check_rotation_matrix.cpp
    math/pose/invert.cpp
    math/pose/orthogonalize.cpp

    math/periodic/periodic_diff.cpp
    math/periodic/periodic_mean.cpp

    math/SoftMinMax.cpp

    shapes/AxisAlignedBoundingBox.cpp
)

SET(INCLUDES
    SimoxUtility.h

    preprocessor/variadic_for_each.h
    preprocessor/drop_front.h

    algorithm/for_each_if.h
    algorithm/apply.hpp

    color/cmaps/colormaps.h
    color/cmaps/Named.h
    color/convert_byte_float.hpp
    color/Color.h
    color/ColorMap.h
    color/GlasbeyLUT.h
    color/hsv.h
    color/interpolation.h
    color/json.h

    EigenStdVector.h
    eigen/EigenStdVector.h
    eigen/AssertAsException.h

    error/SimoxError.h

    filesystem/list_directory.h
    filesystem/remove_trailing_separator.h

    json.h
    json/converters.h
    json/eigen_conversion.h
    json/io.h
    json/json.hpp

    math/convert/rad_to_deg.h
    math/convert/deg_to_rad.h

    math/convert/aa_to_mat3f.h
    math/convert/aa_to_mat4f.h
    math/convert/aa_to_quat.h
    math/convert/aa_to_rpy.h

    math/convert/mat3f_to_aa.h
    math/convert/mat3f_to_mat4f.h
    math/convert/mat3f_to_quat.h
    math/convert/mat3f_to_rpy.h

    math/convert/mat4f_to_aa.h
    math/convert/mat4f_to_mat3f.h
    math/convert/mat4f_to_pos.h
    math/convert/mat4f_to_quat.h
    math/convert/mat4f_to_rpy.h

    math/convert/pos_aa_to_mat4f.h
    math/convert/pos_mat3f_to_mat4f.h
    math/convert/pos_quat_to_mat4f.h
    math/convert/pos_rpy_to_mat4f.h
    math/convert/pos_to_mat4f.h

    math/convert/quat_to_aa.h
    math/convert/quat_to_mat3f.h
    math/convert/quat_to_mat4f.h
    math/convert/quat_to_rpy.h

    math/convert/rpy_to_aa.h
    math/convert/rpy_to_mat3f.h
    math/convert/rpy_to_mat4f.h
    math/convert/rpy_to_quat.h

    math/convert/mat4f_to_xyyaw.h
    math/convert/xyyaw_to_mat4f.h

    math/distance/delta_angle.h
    math/distance/angle_between.h

    math/mean.h

    math/compare/is_equal.h
    math/compare/is_greater.h
    math/compare/is_greater_equal.h
    math/compare/is_inequal.h
    math/compare/is_less.h
    math/compare/is_less_equal.h
    math/compare/value_in_limits_of.h

    math/norm/norm_clamp.h
    math/norm/norm_max.h
    math/norm/norm_min.h

    math/normal/orthogonal_vector.h
    math/normal/normal_to_mat3.h
    math/normal/normal_to_mat4.h

    math/periodic_clamp.h  # Legacy header.
    math/periodic/periodic_clamp.h
    math/periodic/periodic_diff.h
    math/periodic/periodic_mean.h

    math/pose/align_box_orientation.h
    math/pose/check_rotation_matrix.h
    math/pose/is_homogeneous_transform.h
    math/pose/invert.h
    math/pose/orthogonalize.h
    math/pose/pose.h
    math/pose/transform.h

    math/similarity/cosine_similarity.h
    math/similarity/angular_similarity.h

    math/sum.h

    math/SoftMinMax.h
    math/rescale.h

    math/zero.h

    meta/undefined_t/undefined_t.h
    meta/undefined_t/is_set.h
    meta/eigen/enable_if_compile_time_size.h
    meta/eigen/is_eigen_array.h
    meta/eigen/is_eigen_matrix.h
    meta/has_member_macros/has_member.hpp
    meta/type_name.h
    meta/boost_hana/has_hana_accessor.h
    meta/boost_hana/hana_member_name.h
    meta/type_traits/is_any_of.h
    meta/type_traits/is_std_array.h
    meta/type_traits/are_arithmetic.h
    meta/type_traits/is_string_like.h

    meta/EnumNames.hpp
    meta/enum/adapt_enum.h
    meta/enum/EnumNames.hpp

    shapes/AxisAlignedBoundingBox.h
    shapes/OrientedBoxBase.h
    shapes/OrientedBox.h
    shapes/XYConstrainedOrientedBox.h
)

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


SET_TARGET_PROPERTIES(SimoxUtility PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
SET_TARGET_PROPERTIES(SimoxUtility PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
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
