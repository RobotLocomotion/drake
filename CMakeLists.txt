cmake_minimum_required(VERSION 2.8.6) # for UseJava.cmake

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# set up pods
set(POD_NAME drake)
include(cmake/pods.cmake)

option(BUILD_SHARED_LIBS "Build Drake with shared libraries." ON)
# set up Java, use a target of 1.6
find_package(Java REQUIRED)
include(UseJava)
set(CMAKE_JAVA_COMPILE_FLAGS ${CMAKE_JAVA_COMPILE_FLAGS} -source 6 -target 6)

if (CMAKE_COMPILER_IS_GNUCC)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wuninitialized -Wunused-variable") # -Wunused-parameter")

   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                OUTPUT_VARIABLE GCC_VERSION)
   if (NOT (GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3))
        message(FATAL_ERROR "requires gcc version >= 4.3")  # to support the c++0x flag below
   elseif(GCC_VERSION VERSION_LESS 4.7)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wuninitialized -Wunused-variable") # -Wunused-parameter")

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 if (APPLE)  # this was a step towards getting things to work with
#   clang on mac, but ultimately we didn't get there...   (but I would
#   be worried about sharing pointers between objects compiled against
#   different c++ libs, so removing it)
#    # http://stackoverflow.com/questions/13445742/apple-and-shared-ptr
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
elseif (MSVC)
#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:SSE2 /openmp")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2 /openmp")
endif()

# set up matlab build
include(cmake/mex.cmake)

enable_testing()
include(CTest)

set(drake_jar_javafiles util/Transform.java)
set(drake_jar_requires)

# set up and build lcm types
include(cmake/lcmtypes.cmake)

if (LCM_FOUND)
  #add_subdirectory(lcmtypes)
  lcmtypes_build()
  pods_use_pkg_config_classpath(lcm-java)

  list(APPEND drake_jar_javafiles
        util/MyLCMTypeDatabase.java
	util/MessageMonitor.java
	util/CoordinateFrameData.java
	util/LCMCoder.java
	)
  list(APPEND drake_jar_requires lcm-java lcmtypes_drake-java)

  # add lcmgl
  # annoyingly, libbot does not define a classpath variable. there might be a cleaner way to do this
  execute_process(COMMAND pkg-config --variable=prefix bot2-lcmgl-client OUTPUT_VARIABLE LCMGL_BUILD_DIR)
  if (NOT LCMGL_BUILD_DIR)
     message(WARNING "\n\n----- WARNING: bot2-lcmgl not found")
  else ()
    string(STRIP ${LCMGL_BUILD_DIR} LCMGL_BUILD_DIR)
    find_jar(LCMGL_JAR_FILE bot2-lcmgl
  		PATHS "${LCMGL_BUILD_DIR}/share/java/")
    if (NOT LCMGL_JAR_FILE)
       message(FATAL_ERROR "\n\n----- ERROR: bot2-lcmgl not found")
    else()
       message(STATUS "Found bot2-lcmgl")
    endif()
    set(CMAKE_JAVA_INCLUDE_PATH ${CMAKE_JAVA_INCLUDE_PATH}:${LCMGL_JAR_FILE})
    list(APPEND drake_jar_javafiles
 	util/BotLCMGLClient.java
	)
  endif()

endif()

add_jar(drake SOURCES ${drake_jar_javafiles})
install_jar(drake share/java)
pods_install_pkg_config_file(drake-java
    CLASSPATH drake
    REQUIRES ${drake_jar_requires}
    VERSION 0.0.1)

pods_find_pkg_config(eigen3 3.2)

if (eigen3_FOUND)
   pods_use_pkg_config_includes( eigen3 )
else()
   # then use the traditional approach to finding Eigen3
   find_package( Eigen3 3.2 )
   if (Eigen3_FOUND)
      set(eigen3_FOUND 1)
      include_directories( EIGEN3_INCLUDE_DIR )
   endif()
endif()

add_subdirectory(util)
add_subdirectory(solvers)
add_subdirectory(systems)
add_subdirectory(thirdParty/GetFullPath)

# setup matlab pods-compliance
include(cmake/matlab_pods.cmake)
pods_configure_matlab_paths()

get_relative_path("${CMAKE_INSTALL_PREFIX}/matlab" "${CMAKE_CURRENT_SOURCE_DIR}" relpath)

message(STATUS "Writing addpath_drake.m and rmpath_drake.m to ${CMAKE_INSTALL_PREFIX}/matlab")
file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_drake.m
	   "function addpath_drake()\n"
	   "  mfiledir = fileparts(which(mfilename));\n"
	   "  wd = cd(fullfile(mfiledir,'${relpath}'));\n"
	   "  addpath_drake();\n"
	   "  cd(wd);\n"
    )

file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/rmpath_drake.m
	   "function rmpath_drake()\n"
	   "  mfiledir = fileparts(which(mfilename));\n"
	   "  wd = cd(fullfile(mfiledir,'${relpath}'));\n"
	   "  rmpath_drake();\n"
	   "  cd(wd);\n"
    )

find_program(avl avl PATHS ${CMAKE_INSTALL_DIR}/bin )
find_program(xfoil xfoil PATHS ${CMAKE_INSTALL_DIR}/bin )
find_program(ffmpeg ffmpeg)

# todo: build doxygen here (and remove it from the Makefile)
# http://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/


# write some additional information to the CMake cache
set(lcm_java_classpath "${lcm-java_CLASSPATH}" CACHE FILEPATH "${lcm-java_CLASSPATH}" FORCE)

if (APPLE)
  set(DYLD_LIBRARY_PATH "$ENV{DYLD_LIBRARY_PATH}" CACHE STRING "Environment variable used to launch processes from Matlab")
  set(DYLD_FRAMEWORK_PATH "$ENV{DYLD_FRAMEWORK_PATH}" CACHE STRING "Environment variable used to launch processes from Matlab")
  mark_as_advanced(DYLD_LIBRARY_PATH)
  mark_as_advanced(DYLD_FRAMEWORK_PATH)
else()
  set(LD_LIBRARY_PATH "$ENV{LD_LIBRARY_PATH}" CACHE STRING "Environment variable used to launch processes from Matlab")
  mark_as_advanced(LD_LIBRARY_PATH)
endif()

add_test(NAME "RigidBodyManipulatorMemoryTest"
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/examples/Acrobot"
	COMMAND "matlab" #"${CMAKE_SOURCE_DIR}/cmake/matlab_clean.pl"
	"-nosplash" "-nodisplay" "-r" "addpath('${CMAKE_INSTALL_PREFIX}/matlab'); addpath_${POD_NAME}; try, r = RigidBodyManipulator('Acrobot.urdf'); megaclear; catch ex, disp(getReport(ex,'extended')); disp(''); force_close_system; exit(1); end; force_close_system; exit(0)")
set_tests_properties(RigidBodyManipulatorMemoryTest PROPERTIES TIMEOUT 60)
