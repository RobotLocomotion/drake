cmake_minimum_required(VERSION 2.8.6) # for UseJava.cmake

# options
option(BUILD_SHARED_LIBS "Build Drake with shared libraries." ON)
option(RANDOMIZE_UNIT_TESTS "every matlab unit is started with a new random seed" OFF)

#cmake_policy(SET CMP0042 NEW)
if (POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)  # so variables inside quotes are still evaluated in if() statements
endif()

# Set a consistent MACOSX_RPATH default across all CMake versions.
# When CMake 3.0.0 is required, remove this block (see CMP0042).
if (NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH ON)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# set up pods
set(POD_NAME drake)
include(cmake/pods.cmake)

# set up Java, use a target of 1.6
find_package(Java REQUIRED)
include(UseJava)
set(CMAKE_JAVA_COMPILE_FLAGS ${CMAKE_JAVA_COMPILE_FLAGS} -source 6 -target 6)

if (CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wuninitialized -Wunused-variable") # -Wunused-parameter")

  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
    message(FATAL_ERROR "requires gcc version >= 4.7")  # to support the c++0x flag below
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
   if (NOT ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 18.00.40628)
     # version number decoder ring at https://en.wikipedia.org/wiki/Visual_C%2B%2B 
     # at least one user hit a compiler crash with VC++ 12.0, which was resolved by installing the latest service packs.  I don't know that 40629 is required, but know that 00 is not sufficient.
     message(FATAL_ERROR "requires MS VC++ 12.0 update 5 or greater (Visual Studio >= 2013).  download for free at http://visualstudio.com")
   endif()

#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:SSE2 /openmp")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2 /openmp")

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251")
  # disable warning C4251: e.g.'RigidBody::linkname' : class 'std::basic_string<_Elem,_Traits,_Ax>' needs to have dll-interface to be used by clients of class 'RigidBody'
  # followed by template linking errors.  After reading, e.g.:
  #   https://connect.microsoft.com/VisualStudio/feedback/details/696593/vc-10-vs-2010-basic-string-exports
  # I think we're not being sufficiently careful with our library interfaces (on other platforms as well) - Russ

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4503")
  # disable C4503: 'identifier' : decorated name length exceeded, name was truncated
  # these occur due to AutoDiffScalar inputs to DrakeJoint methods, which results in very long type names
  # From https://msdn.microsoft.com/en-us/library/074af4b6.aspx:
  # It is possible to ship an application that generates C4503, but if you get link time errors on a truncated symbol,
  # it will be more difficult to determine the type of the symbol in the error. Debugging will also be more difficult;
  # the debugger will also have difficultly mapping symbol name to type name. The correctness of the program, however,
  # is unaffected by the truncated name.

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4522")
  # disable C4522: 'class' : multiple assignment operators specified
  # The class has multiple assignment operators of a single type. This warning is informational;
  # the constructors are callable in your program.

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996") # disable sprintf security warning

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj") # after receiving RigidBodyManipulator.cpp : fatal error C1128: number of sections exceeded object file format limit: compile with /bigobj
endif()

# set up matlab build
include(cmake/mex.cmake)
mex_setup()

# frustratingly, this didn't seem to work, so instead I've added a -C argument to both the Makefile and the drake-admin/runUnitTests scripts
#if (NOT CTEST_CONFIGURATION_TYPE)
#   set(CTEST_CONFIGURATION_TYPE ${CMAKE_BUILD_TYPE})
#   message("Setting CTEST Configuration Type to ${CTEST_CONFIGURATION_TYPE}")
#endif()

find_package( Perl )
if ( PERL_FOUND )
   file(WRITE ${CMAKE_BINARY_DIR}/CTestCustom.cmake 
   	      "set(CTEST_CUSTOM_PRE_TEST \"${PERL_EXECUTABLE} ${PROJECT_SOURCE_DIR}/cmake/add_matlab_unit_tests.pl ${PROJECT_SOURCE_DIR}\")\n")
endif()

enable_testing()
include(CTest)

set(drake_jar_javafiles util/Transform.java)
set(drake_jar_requires)

include(cmake/procman.cmake)  # helper script for writing procman files

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
   if (EIGEN3_FOUND)
      set(eigen3_FOUND 1)
      include_directories( ${EIGEN3_INCLUDE_DIR} )
   else()
      message(FATAL_ERROR "Could not find eigen, which is a required depedency")
   endif()
endif()

include_directories(thirdParty/spruce/include)


add_subdirectory(util)
add_subdirectory(solvers)
add_subdirectory(systems)
add_subdirectory(thirdParty/GetFullPath)
add_subdirectory(thirdParty/spruce)
add_subdirectory(examples)

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
     "end\n"
     "\n"
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

if (MATLAB_FOUND)
  add_test(NAME "RigidBodyManipulatorMemoryTest"
  	COMMAND "matlab" #"${PROJECT_SOURCE_DIR}/cmake/matlab_clean.pl"
  	"-nosplash" "-nodisplay" "-wait" "-r" "addpath('${CMAKE_INSTALL_PREFIX}/matlab'); addpath_${POD_NAME}; cd('${PROJECT_SOURCE_DIR}/examples/Acrobot'); try, r = RigidBodyManipulator('Acrobot.urdf'); megaclear; catch ex, disp(getReport(ex,'extended')); disp(''); force_close_system; exit(1); end; force_close_system; exit(0)")
  set_tests_properties(RigidBodyManipulatorMemoryTest PROPERTIES TIMEOUT 300)
endif()
