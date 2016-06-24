# TODO: still build something useful if Gurobi does not exist

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # TODO(#2372) These are warnings that we can't handle yet.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")
endif()

pods_find_pkg_config(gurobi)

if(gurobi_FOUND)

  add_library_with_exports(LIB_NAME drakeQP SOURCE_FILES QP.cpp)
  pods_use_pkg_config_packages(drakeQP gurobi)

  if(MATLAB_FOUND)
    add_mex(fastQPmex fastQPmex.cpp)
    target_link_libraries(fastQPmex drakeQP)
    add_mex(gurobiQPmex gurobiQPmex.cpp)
    target_link_libraries(gurobiQPmex drakeQP)
  endif()

  pods_install_libraries(drakeQP)
  drake_install_headers(fastQP.h gurobiQP.h)
  pods_install_pkg_config_file(drake-qp
    LIBS -ldrakeQP
    REQUIRES gurobi
    VERSION 0.0.1)

endif()

if(MATLAB_FOUND)
  add_mex(lqrmex lqrmex.cpp)
  target_link_libraries(lqrmex drakeUtil)
endif()

pods_find_pkg_config(mosek)

set(optimization_files)
list(APPEND optimization_files MathematicalProgram.cpp MobyLCP.cpp system_identification.cpp)
pods_find_pkg_config(ipopt)
if (ipopt_FOUND)
  list(APPEND optimization_files IpoptSolver.cpp)
else()
  list(APPEND optimization_files NoIpopt.cpp)
endif(ipopt_FOUND)
pods_find_pkg_config(nlopt)
if(nlopt_FOUND)
  list(APPEND optimization_files NloptSolver.cpp)
else()
  list(APPEND optimization_files NoNlopt.cpp)
endif()
pods_find_pkg_config(snopt_c)
if(snopt_c_FOUND)
  list(APPEND optimization_files SnoptSolver.cpp)
else()
  list(APPEND optimization_files NoSnopt.cpp)
endif()

#if(mosek_FOUND)
#  message("FOUND IT!!")
#  list(APPEND optimization_files mosekLP.cpp)
#endif()


if(mosek_FOUND)
  set(MOSEK_PLATFORM_PATH "${CMAKE_SOURCE_DIR}/externals/mosek/pod-build/mosek/7/tools/platform/${PLATFORM_NAME}")
  add_library_with_exports(LIB_NAME mosekLP SOURCE_FILES mosekLP.cpp)
  # Check for the mosek license file
  if (NOT EXISTS $ENV{HOME}/mosek/mosek.lic)
      message(WARNING "You do not appear to have a license for mosek installed in $ENV{HOME}/mosek/mosek.lic\n If you are an academic user, you can open the following url in your favorite browser and request the license:\n     http://license.mosek.com/license2/academic/\n Then check your email for the license file and put it in $ENV{HOME}/mosek/mosek.lic\n Otherwise visit https://mosek.com/resources/trial-license to request a trial license.")
  endif()
  pods_use_pkg_config_packages(mosekLP mosek)
  #pods_install_headers(${MOSEK_PLATFORM_PATH}/h/mosek.h DESTINATION mosek)
  #drake_install_headers(mosekLP.h)
  pods_install_libraries(mosekLP)
endif()


add_library_with_exports(LIB_NAME drakeOmakeptimization SOURCE_FILES ${optimization_files})
target_link_libraries(drakeOptimization drakePolynomial)
if(mosek_FOUND)
  target_link_libraries(drakeOptimization mosekLP)
endif()
drake_install_headers(MathematicalProgram.h Optimization.h system_identification.h)
pods_install_libraries(drakeOptimization)
pods_install_pkg_config_file(drake-optimization
  LIBS -ldrakeOptimization
  REQUIRES
  VERSION 0.0.1)



if(ipopt_FOUND)
  pods_use_pkg_config_packages(drakeOptimization ipopt)
  # IPOPT builds its own version of BLAS during compilation, and links
  # it into the output library.  On (at least some) ELF systems, you
  # wind up with a symbol name collision between IPOPT's BLAS and
  # other BLAS implementations in other dynamic libraries (MATLAB, I'm
  # looking in your direction...)  Since the actual implementations
  # are incompatible, this does not end well.  Linking with the
  # -Bsymbolic flags causes the internal references to these symbols
  # inside libdrakeOptimization to be handled at compile time, and
  # prevents the dynamic linker from hijacking IPOPT's calls into
  # BLAS.
  #
  # This should probably be a test for the GNU linker instead of a platform.
  if (NOT (APPLE OR WIN32))
    set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic -Wl,-Bsymbolic-functions")
  endif()
endif()
if(nlopt_FOUND)
  pods_use_pkg_config_packages(drakeOptimization nlopt)
endif()
if(snopt_c_FOUND)
  pods_use_pkg_config_packages(drakeOptimization snopt_c)
endif()

if(snopt_c_FOUND AND MATLAB_FOUND)
  add_mex(NonlinearProgramSnoptmex NonlinearProgramSnoptmex.cpp)
  pods_use_pkg_config_packages(NonlinearProgramSnoptmex snopt_c)
endif()

add_subdirectory(qpSpline)
add_subdirectory(test)
add_subdirectory(+drakeFunction)
add_subdirectory(BMI)
add_subdirectory(trajectoryOptimization/test)
