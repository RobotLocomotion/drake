add_library_with_exports(LIB_NAME drakeAtlasUtil SOURCE_FILES atlasUtil.cpp)
pods_install_libraries(drakeAtlasUtil)
drake_install_headers(atlasUtil.h)
pods_install_pkg_config_file(drake-atlas-util
  LIBS -ldrakeAtlasUtil
  REQUIRES
  VERSION 0.0.1)

add_matlab_test(NAME examples/Atlas/robotiqHandInspector COMMAND robotiqHandInspector)
add_matlab_test(NAME examples/Atlas/runAtlasDynamics COMMAND runAtlasDynamics)
add_matlab_test(NAME examples/Atlas/runAtlasFootstepPlanning COMMAND runAtlasFootstepPlanning)
add_matlab_test(NAME examples/Atlas/runAtlasFrontalDynamics COMMAND runAtlasFrontalDynamics)
add_matlab_test(NAME examples/Atlas/runAtlasJumping COMMAND runAtlasJumping)
add_matlab_test(NAME examples/Atlas/runAtlasSagittalDynamics COMMAND runAtlasSagittalDynamics)
add_matlab_test(NAME examples/Atlas/runAtlasWalkingPlanning COMMAND runAtlasWalkingPlanning)
add_matlab_test(NAME examples/Atlas/runCOMFixedPointSearch COMMAND runCOMFixedPointSearch)
add_matlab_test(NAME examples/Atlas/runDRCDoorTask COMMAND runDRCDoorTask)
add_matlab_test(NAME examples/Atlas/runRobotiqPDControl COMMAND runRobotiqPDControl)

if (LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/Atlas/runAtlasBalancing COMMAND runAtlasBalancing)
  add_matlab_test(NAME examples/Atlas/runAtlasBalancingPerturb COMMAND runAtlasBalancingPerturb)
  add_matlab_test(NAME examples/Atlas/runAtlasBalancingWithContactSensor COMMAND runAtlasBalancingWithContactSensor)
  add_matlab_test(NAME examples/Atlas/runAtlasHandControl COMMAND runAtlasHandControl)
  add_matlab_test(NAME examples/Atlas/runAtlasManip COMMAND runAtlasManip)
  add_matlab_test(NAME examples/Atlas/runAtlasWalking COMMAND runAtlasWalking)
  add_matlab_test(NAME examples/Atlas/runDRCPracticeTerrain COMMAND runDRCPracticeTerrain PROPERTIES  TIMEOUT  3000)
  add_matlab_test(NAME examples/Atlas/runRobotiqTendons COMMAND runRobotiqTendons)
endif()

add_subdirectory(test)
