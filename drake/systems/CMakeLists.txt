
if (simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()

include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/util)
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)

add_subdirectory(frames)
add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

pods_install_headers(System.h LinearSystem.h Simulation.h LQR.h DESTINATION drake)

add_subdirectory(test)
