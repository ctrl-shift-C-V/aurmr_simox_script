if (qwt_INCLUDE_DIR AND qwt_LIBRARIES)
    # in cache already
    set(qwt_FOUND TRUE)
else()
    if (NOT qwt_DIR)
        set(qwt_DIR $ENV{QWT_DIR})
    endif (NOT qwt_DIR)
    find_path(qwt_INCLUDE_DIR qwt.h
        # installation selected by user
        ${qwt_DIR}
        ${qwt_DIR}/src
        ${qwt_DIR}/include
        /usr/include/qwt
    )

    message(STATUS "qwt_DIR: ${qwt_DIR}")
    message(STATUS "qwt-Include: ${qwt_INCLUDE_DIR}")

    set(qwtlibname qwt-qt5)

    find_library(qwt_LIBRARY
                 NAMES ${qwtlibname}
                 PATHS ${qwt_DIR}/lib)
    message(STATUS "qwt-lib: ${qwt_LIBRARY}")

    set(qwt_LIBRARIES ${qwt_LIBRARY})
    set(qwt_INCLUDE_DIRS ${qwt_INCLUDE_DIR})

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(qwt DEFAULT_MSG qwt_INCLUDE_DIR qwt_LIBRARIES)
    set(qwt_FOUND ${QWT_FOUND})

    mark_as_advanced(qwt_INCLUDE_DIR qwt_LIBRARIES)

endif()
