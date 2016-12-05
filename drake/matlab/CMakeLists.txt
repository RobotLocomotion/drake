configure_file(addpath_drake.m.in addpath_drake.m @ONLY)
configure_file(rmpath_drake.m.in rmpath_drake.m @ONLY)

configure_file(get_drake_binary_dir.m.in get_drake_binary_dir.m @ONLY)
configure_file(get_drake_install_dir.m.in get_drake_install_dir.m @ONLY)

configure_file(drake_get_base_path.m.in drake_get_base_path.m @ONLY)
configure_file(drake_get_bin_path.m.in drake_get_bin_path.m @ONLY)
configure_file(drake_get_lib_path.m.in drake_get_lib_path.m @ONLY)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/addpath_drake.m"
  "${CMAKE_CURRENT_BINARY_DIR}/rmpath_drake.m"
  "${CMAKE_CURRENT_BINARY_DIR}/get_drake_binary_dir.m"
  "${CMAKE_CURRENT_BINARY_DIR}/get_drake_install_dir.m"
  "${CMAKE_CURRENT_BINARY_DIR}/drake_get_base_path.m"
  "${CMAKE_CURRENT_BINARY_DIR}/drake_get_bin_path.m"
  "${CMAKE_CURRENT_BINARY_DIR}/drake_get_lib_path.m"
  DESTINATION matlab)

find_program(FFMPEG_EXECUTABLE NAMES ffmpeg avconv
  DOC "Path to the ffmpeg executable")

if(avl_FOUND)
  # Needs to be in cache to keep MATLAB happy. AVL_EXECUTABLE is set by
  # find_package(avl).
  set(AVL_EXECUTABLE "${AVL_EXECUTABLE}"
    CACHE FILEPATH "Path to the avl executable")
endif()

if(xfoil_FOUND)
  # Needs to be in cache to keep MATLAB happy. XFOIL_EXECUTABLE is set by
  # find_package(xfoil).
  set(XFOIL_EXECUTABLE "${XFOIL_EXECUTABLE}"
    CACHE FILEPATH "Path to the xfoil executable")
endif()

add_subdirectory(util)
add_subdirectory(solvers)
add_subdirectory(systems)

if(BUILD_TESTING)
  drake_add_matlab_test(NAME "matlab/RigidBodyManipulatorMemoryTest"
    OPTIONAL bullet
    COMMAND "r = RigidBodyManipulator('Acrobot.urdf'), megaclear"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Acrobot SIZE large)

  function(drake_add_matlab_addpath_test dependency)
    drake_add_matlab_test(NAME matlab/addpath_${dependency}_test
      REQUIRES ${dependency} COMMAND "addpath_${dependency}" SIZE small)
  endfunction()

  drake_add_matlab_addpath_test(gurobi)
  drake_add_matlab_addpath_test(mosek)
  drake_add_matlab_addpath_test(sedumi)
  drake_add_matlab_addpath_test(snopt)
  drake_add_matlab_addpath_test(spotless)
  drake_add_matlab_addpath_test(yalmip)
endif()
