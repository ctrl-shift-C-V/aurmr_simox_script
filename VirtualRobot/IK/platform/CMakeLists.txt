armarx_component_set_name("Armar6MecanumPlatform")
armarx_set_target("Library: Armar6MecanumPlatform")

set(LIB_NAME       Armar6MecanumPlatform)

set(LIBS ArmarXCore SimoxUtility)

set(LIB_FILES Armar6MecanumPlatform.cpp)
set(LIB_HEADERS Armar6MecanumPlatform.h)
armarx_add_library("${LIB_NAME}" "${LIB_FILES}" "${LIB_HEADERS}" "${LIBS}")

# add unit tests
add_subdirectory(test)
