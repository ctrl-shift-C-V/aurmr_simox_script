
ADD_SU_TEST( OrientedBoxTest )
ADD_SU_TEST( XYConstrainedOrientedBoxTest )
ADD_SU_TEST( JsonConversionsTest )


SET(TEST_FILES
    AABB_center_extents.json
    AABB_min_max.json
    OrientedBox_pos_ori_dimensions.json
    OrientedBox_position_orientation_extents.json
)

foreach (FILENAME IN LISTS TEST_FILES)
    configure_file("${FILENAME}" "${Simox_TEST_DIR}/${FILENAME}" COPYONLY)
endforeach()
