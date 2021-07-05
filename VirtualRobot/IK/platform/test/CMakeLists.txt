
# Libs required for the tests
SET(LIBS ${LIBS} ArmarXCore Armar6MecanumPlatform)
 
armarx_add_test(Armar6MecanumPlatformTest Armar6MecanumPlatformTest.cpp "${LIBS}")