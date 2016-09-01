add_library_with_exports(LIB_NAME drakeAtlasUtil SOURCE_FILES atlasUtil.cpp)
pods_install_libraries(drakeAtlasUtil)
drake_install_headers(
        atlasUtil.h
        atlas_plant.h
        )
pods_install_pkg_config_file(drake-atlas-util
  LIBS -ldrakeAtlasUtil
  REQUIRES
  VERSION 0.0.1)

if (lcm_FOUND)
  add_executable(atlas_dynamics_demo atlas_dynamics_demo.cc atlas_plant.cc)
  target_link_libraries(atlas_dynamics_demo drakeRBSystem drakeLCMSystem)
  add_dependencies(atlas_dynamics_demo drake_lcmtypes drake_lcmtypes_hpp)
  pods_use_pkg_config_packages(atlas_dynamics_demo lcm)
  add_test(NAME atlas_dynamics_demo COMMAND atlas_dynamics_demo 0.4)
endif()

add_matlab_test(NAME examples/Atlas/robotiqHandInspector OPTIONAL bullet COMMAND robotiqHandInspector)
add_matlab_test(NAME examples/Atlas/runAtlasDynamics OPTIONAL bullet gurobi snopt COMMAND runAtlasDynamics)
add_matlab_test(NAME examples/Atlas/runAtlasFootstepPlanning REQUIRES gurobi lcm libbot OPTIONAL bullet snopt COMMAND runAtlasFootstepPlanning)
add_matlab_test(NAME examples/Atlas/runAtlasFrontalDynamics OPTIONAL bullet gurobi snopt COMMAND runAtlasFrontalDynamics)
add_matlab_test(NAME examples/Atlas/runAtlasJumping REQUIRES spotless OPTIONAL bullet gurobi snopt COMMAND runAtlasJumping)
add_matlab_test(NAME examples/Atlas/runAtlasSagittalDynamics OPTIONAL bullet gurobi snopt COMMAND runAtlasSagittalDynamics)
add_matlab_test(NAME examples/Atlas/runAtlasWalkingPlanning REQUIRES gurobi lcm libbot OPTIONAL bullet snopt COMMAND runAtlasWalkingPlanning)
add_matlab_test(NAME examples/Atlas/runCOMFixedPointSearch OPTIONAL bullet gurobi COMMAND runCOMFixedPointSearch)

# Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
# add_matlab_test(NAME examples/Atlas/runDRCDoorTask REQUIRES iris mosek COMMAND runDRCDoorTask)

add_matlab_test(NAME examples/Atlas/runRobotiqPDControl OPTIONAL bullet gurobi snopt COMMAND runRobotiqPDControl)

if(LONG_RUNNING_TESTS)

  if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND UNIX AND NOT APPLE) # FIXME: see #3147.
    add_matlab_test(NAME examples/Atlas/runAtlasBalancingPerturb REQUIRES gurobi lcm libbot COMMAND runAtlasBalancingPerturb PROPERTIES TIMEOUT 1500)
    add_matlab_test(NAME examples/Atlas/runAtlasManip REQUIRES gurobi lcm libbot COMMAND runAtlasManip PROPERTIES TIMEOUT 1500)
  endif()

  # Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
  # add_matlab_test(NAME examples/Atlas/runAtlasWalking REQUIRES gurobi lcm COMMAND runAtlasWalking PROPERTIES TIMEOUT 1500)

  # add_matlab_test(NAME examples/Atlas/runAtlasBalancing REQUIRES gurobi COMMAND runAtlasBalancing PROPERTIES TIMEOUT 1500)  # FIXME: see #2839
  # add_matlab_test(NAME examples/Atlas/runAtlasBalancingWithContactSensor REQUIRES gurobi COMMAND runAtlasBalancingWithContactSensor PROPERTIES TIMEOUT 1500)  # FIXME: see #2839
  # add_matlab_test(NAME examples/Atlas/runAtlasHandControl REQUIRES gurobi COMMAND runAtlasHandControl PROPERTIES TIMEOUT 3000)  # FIXME: see #2839

  add_matlab_test(NAME examples/Atlas/runRobotiqTendons COMMAND runRobotiqTendons PROPERTIES TIMEOUT 1500)

  # Deactivated due to excessive timeouts.
  # add_matlab_test(NAME examples/Atlas/runDRCPracticeTerrain REQUIRES iris mosek COMMAND runDRCPracticeTerrain PROPERTIES TIMEOUT 3000)  # FIXME

endif()

add_subdirectory(test)
