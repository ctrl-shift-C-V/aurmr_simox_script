
ADD_SU_TEST( AxisAlignedBoundingBoxTest )
ADD_SU_TEST( JsonConversionsTest )
ADD_SU_TEST( OrientedBoxTest )
ADD_SU_TEST( XYConstrainedOrientedBoxTest )


SET(TEST_FILES
    AABB_center_extents.json
    AABB_min_max.json
    OrientedBox_pos_ori_dimensions.json
    OrientedBox_position_orientation_extents.json
)

foreach (FILENAME IN LISTS TEST_FILES)
    configure_file("${FILENAME}" "${Simox_BINARY_DIR}/SimoxUtility/tests/shapes/${FILENAME}" COPYONLY)
endforeach()
