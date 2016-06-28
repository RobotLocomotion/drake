cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0025 NEW)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0054 OLD)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
  message(FATAL_ERROR "GCC version must be at least 4.9")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7)
  message(FATAL_ERROR "Apple Clang version must be at least 7")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.7)
  message(FATAL_ERROR "Clang version must be at least 3.7")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19)
  message(FATAL_ERROR "MSVC version must be at least 14")
endif()

# Create a cache variable that contains the
# max number of concurent link jobs that can be run
# in a Ninja build, if the value is 0 then let Ninja
# use as many as it wants.
set(CMAKE_NINJA_LINK_POOL_SIZE 0 CACHE STRING
  "Number of concurent link jobs that can be run with Ninja build tool")
# if the link pool is non-zero then set the property
# on all the targets in drake
if(CMAKE_NINJA_LINK_POOL_SIZE GREATER 0)
  # Ninja: Configure a job pool to limit simultaneous linking.
  set_property(GLOBAL PROPERTY JOB_POOLS link_pool=${CMAKE_NINJA_LINK_POOL_SIZE})
  # Ninja: Assign all linking to our link job pool.
  set(CMAKE_JOB_POOL_LINK link_pool)
endif()

# options
option(BUILD_SHARED_LIBS "Build Drake with shared libraries." ON)
option(LONG_RUNNING_TESTS "some tests should be run nightly for coverage, but are too slow for CI" OFF)
option(RANDOMIZE_MATLAB_TESTS "every MATLAB unit is started with a new random seed" OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# prepend module path so we can override FindMatlab.cmake
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
if(WITH_PYTHON_3)
  set(Python_ADDITIONAL_VERSIONS 3.5)
  find_package(PythonInterp 3)
else()
  find_package(PythonInterp)
endif()

# set up pods
set(POD_NAME drake)
include(cmake/pods.cmake)

find_package(Java 1.6 REQUIRED)
include(UseJava)

function(drake_install_headers)
  file(RELATIVE_PATH rel_path ${PROJECT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR})
  pods_install_headers(${ARGV} DESTINATION drake/${rel_path})
endfunction()

# set up automatic export header generation
# export headers will be installed to build/include/drake, but are created in ${PROJECT_BINARY_DIR}/exports/drake
# This means that export headers can be included as drake/bla_export.h
include(GenerateExportHeader)
function(add_library_with_exports)
  set(options STATIC)
  set(oneValueArgs LIB_NAME)
  set(multiValueArgs SOURCE_FILES)
  cmake_parse_arguments(parsed_args "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(parsed_args_STATIC)
    add_library(${parsed_args_LIB_NAME} STATIC ${parsed_args_SOURCE_FILES})
    # Add a public preprocessor macro definition for LIB_NAME_STATIC_DEFINE,
    # which suppresses platform-specific shared library export declarations
    # in this target and all targets that depend on it.
    string(TOUPPER ${parsed_args_LIB_NAME} capitalized_lib_name)
    set(static_define ${capitalized_lib_name}_STATIC_DEFINE)
    target_compile_definitions(${parsed_args_LIB_NAME} PUBLIC ${static_define})
  else()
    add_library(${parsed_args_LIB_NAME} SHARED ${parsed_args_SOURCE_FILES})
  endif()

  set(exports_abs_path ${PROJECT_BINARY_DIR}/exports/drake/${parsed_args_LIB_NAME}_export.h)
  generate_export_header(${parsed_args_LIB_NAME} EXPORT_FILE_NAME ${exports_abs_path})
  install(FILES ${exports_abs_path} DESTINATION include/drake/)
endfunction()
include_directories(BEFORE ${PROJECT_BINARY_DIR}/exports/)
include_directories(BEFORE ${PROJECT_SOURCE_DIR}/..)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=all -Werror=ignored-qualifiers")
elseif(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251")
  # disable warning C4251: e.g.'RigidBody::linkname' : class 'std::basic_string<_Elem,_Traits,_Ax>' needs to have dll-interface to be used by clients of class 'RigidBody'
  # followed by template linking errors. After reading, e.g.:
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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj") # after receiving RigidBodyTree.cpp : fatal error C1128: number of sections exceeded object file format limit: compile with /bigobj

  add_definitions(-D_USE_MATH_DEFINES) # to get math constants on MSVC (see https://msdn.microsoft.com/en-us/library/4hwaceh6.aspx)
endif()

# similar to the export headers, any other generated headers will be installed to build/include/drake, but will be created in ${PROJECT_BINARY_DIR}/exports/drake
# These headers can be included as drake/bla.h
set(generated_dir ${PROJECT_BINARY_DIR}/generated)
set(drake_path_header ${generated_dir}/drake/Path.h)
configure_file(core/Path.h.in ${drake_path_header})
install(FILES ${drake_path_header} DESTINATION include/drake/)
include_directories(BEFORE ${generated_dir})

# set up matlab build
include(cmake/mex.cmake)
if(DISABLE_MATLAB)
  message(STATUS "MATLAB is disabled because the CMake option DISABLE_MATLAB is set to ON.")
else()
  # Search for MATLAB and configure various MEX options.
  # If MATLAB is found, MATLAB_FOUND is set.
  mex_setup()

  # if MATLAB is in the PATH then use cmake find_package to find it
  # along with what is needed to use matlab_add_unit_test
  if(MATLAB_FOUND)
    find_package(Matlab COMPONENTS MAIN_PROGRAM REQUIRED)

    set(Matlab_Java_MAIN_PROGRAM_ARGS -nodesktop -nodisplay -nosplash)
    if(WIN32)
      set(Matlab_Java_MAIN_PROGRAM_ARGS ${Matlab_Java_MAIN_PROGRAM_ARGS} -wait)
    endif()
    execute_process(
      COMMAND ${Matlab_MAIN_PROGRAM} ${Matlab_Java_MAIN_PROGRAM_ARGS} -r "version -java,quit"
      RESULT_VARIABLE Matlab_Java_RESULT_VARIABLE
      ERROR_VARIABLE Matlab_Java_OUTPUT_VARIABLE
      OUTPUT_VARIABLE Matlab_Java_OUTPUT_VARIABLE
      TIMEOUT 180)
    if(NOT Matlab_Java_RESULT_VARIABLE AND Matlab_Java_OUTPUT_VARIABLE MATCHES "Java ([0-9]+\\.[0-9]+)\\.[0-9_.]+")
      set(Matlab_Java_VERSION_MAJOR_MINOR ${CMAKE_MATCH_1})
      set(Java_VERSION_MAJOR_MINOR "${Java_VERSION_MAJOR}.${Java_VERSION_MINOR}")
      if(NOT Matlab_Java_VERSION_MAJOR_MINOR VERSION_EQUAL Java_VERSION_MAJOR_MINOR)
        message(WARNING "MATLAB JVM version (${Matlab_Java_VERSION_MAJOR_MINOR}) does not match installed Java SDK version (${Java_VERSION_MAJOR_MINOR})")
        if(Matlab_Java_VERSION_MAJOR_MINOR VERSION_LESS Java_VERSION_MAJOR_MINOR)
          set(CMAKE_JAVA_COMPILE_FLAGS -source ${Matlab_Java_VERSION_MAJOR_MINOR} -target ${Matlab_Java_VERSION_MAJOR_MINOR})
        endif()
      endif()
    else()
      message(WARNING "Could not determine MATLAB JVM version")
    endif()
  endif()
endif()

# add ctest for a MATLAB string
# add_matlab_test(NAME <testname> COMMAND <matlab_string_to_eval>
#   [CONFIGURATIONS <config>...]
#   [WORKING_DIRECTORY <dir>]
#   [PROPERTIES <prop1> <value1> ...]
#   [REQUIRES <package1> <package2> ...])
# note: default working directory is CMAKE_CURRENT_SOURCE_DIR
# note: properties are set internally using set_tests_properties (<testname> PROPERTIES <prop1> <value1> ...)
# note: packages in REQUIRES must write a file addpath_<package>.m to CMAKE_INSTALL_PREFIX/matlab
function(add_matlab_test)
  # no matlab no tests
  if(NOT MATLAB_FOUND)
    return()
  endif()
  set(oneValueArgs NAME COMMAND WORKING_DIRECTORY)
  set(multiValueArgs PROPERTIES REQUIRES)
  cmake_parse_arguments(parsed_args "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(parsed_args_REQUIRES)
    foreach(require ${parsed_args_REQUIRES})
      if(NOT EXISTS "${CMAKE_INSTALL_PREFIX}/matlab/addpath_${require}.m")
        message(STATUS "Not running ${parsed_args_NAME} because ${require} was not installed.")
        return()
      endif()
    endforeach()
  endif()

  string(REPLACE ' '' parsed_args_COMMAND ${parsed_args_COMMAND}) # turn ' into '' so we can eval it in MATLAB
  if(NOT parsed_args_WORKING_DIRECTORY)
    set(parsed_args_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  set(exit_status "~strncmp(ex.identifier,'Drake:MissingDependency',23)")  # missing dependency => pass
  set(additional_paths "${CMAKE_INSTALL_PREFIX}/matlab;${parsed_args_WORKING_DIRECTORY};")
  set(cmd_to_run_before_test "addpath_drake; global g_disable_visualizers; g_disable_visualizers=true;")
  set(custom_Matlab_test_command "try, eval('${parsed_args_COMMAND}'); catch ex, disp(getReport(ex,'extended')); disp(' '); force_close_system; exit(${exit_status}); end; force_close_system; exit(0)")

  if(RANDOMIZE_MATLAB_TESTS)
    set(custom_Matlab_test_command "rng('shuffle'); rng_state=rng; disp(sprintf('To reproduce this test use rng(%d,''%s'')',rng_state.Seed,rng_state.Type)); disp(' '); ${custom_Matlab_test_command}")
  endif()

  matlab_add_unit_test(NAME ${parsed_args_NAME}
    ADDITIONAL_PATH ${additional_paths}
    UNITTEST_PRECOMMAND ${cmd_to_run_before_test}
    CUSTOM_TEST_COMMAND \"${custom_Matlab_test_command}\"
    WORKING_DIRECTORY ${parsed_args_WORKING_DIRECTORY}
    TIMEOUT -1) # use an infinite timeout and rely on add_test ctest timeout

  if(parsed_args_PROPERTIES)
    set_tests_properties(${parsed_args_NAME} PROPERTIES ${parsed_args_PROPERTIES})
  endif()
endfunction()

enable_testing()
include(CTest)
set(GTEST_ROOT ${CMAKE_INSTALL_PREFIX})
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGTEST_DONT_DEFINE_FAIL=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGTEST_DONT_DEFINE_SUCCEED=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGTEST_DONT_DEFINE_TEST=1")
if(WIN32)
  # The following compiler flag is needed to support parameterized Google tests on Windows.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGTEST_LINKED_AS_SHARED_LIBRARY=1")
endif()

set(drake_jar_javafiles util/Transform.java)
set(drake_jar_requires)

include(cmake/procman.cmake)  # helper script for writing procman files

# set up and build lcm types
include(cmake/lcmtypes.cmake)

if(LCM_FOUND)
  #add_subdirectory(lcmtypes)
  lcmtypes_build()
  pods_use_pkg_config_classpath(lcm-java)

  list(APPEND drake_jar_javafiles
    util/MyLCMTypeDatabase.java
    util/MessageMonitor.java
    util/CoordinateFrameData.java
    util/LCMCoder.java)
  list(APPEND drake_jar_requires lcm-java lcmtypes_drake-java)

  # add lcmgl
  # annoyingly, libbot does not define a classpath variable. there might be a cleaner way to do this
  execute_process(COMMAND pkg-config --variable=prefix bot2-lcmgl-client OUTPUT_VARIABLE LCMGL_BUILD_DIR)
  if(NOT LCMGL_BUILD_DIR)
    message(WARNING "\n\n----- WARNING: bot2-lcmgl not found")
  else()
    string(STRIP ${LCMGL_BUILD_DIR} LCMGL_BUILD_DIR)
    find_jar(LCMGL_JAR_FILE bot2-lcmgl PATHS "${LCMGL_BUILD_DIR}/share/java/")
    if(NOT LCMGL_JAR_FILE)
      message(FATAL_ERROR "\n\n----- ERROR: bot2-lcmgl not found")
    else()
      message(STATUS "Found bot2-lcmgl")
    endif()
    set(CMAKE_JAVA_INCLUDE_PATH ${CMAKE_JAVA_INCLUDE_PATH}:${LCMGL_JAR_FILE})
    list(APPEND drake_jar_javafiles util/BotLCMGLClient.java)
  endif()
endif()

add_jar(drake SOURCES ${drake_jar_javafiles})
install_jar(drake share/java)
pods_install_pkg_config_file(drake-java
  CLASSPATH drake
  REQUIRES ${drake_jar_requires}
  VERSION 0.0.1)

pods_find_pkg_config(eigen3 3.2.92)

if(eigen3_FOUND)
  pods_use_pkg_config_includes(eigen3)
else()
  # then use the traditional approach to finding Eigen3
  find_package(Eigen3 3.2.92)
  if(EIGEN3_FOUND)
    set(eigen3_FOUND 1)
    include_directories(${EIGEN3_INCLUDE_DIR})
  else()
    message(FATAL_ERROR "Could not find eigen, which is a required depedency")
  endif()
endif()

# Setup ROS if it is available on the local system
find_package(roscpp QUIET)
if(roscpp_FOUND)
  set(ROS_FOUND ON)
  message(STATUS "Found roscpp installed at ${roscpp_DIR}, so building ROS-specific functionality.")
  include_directories(${roscpp_INCLUDE_DIRS})

  find_package(catkin REQUIRED)
  find_package(genmsg REQUIRED)

  find_package(std_msgs REQUIRED)
  include_directories(${std_msgs_INCLUDE_DIRS})

  find_package(sensor_msgs REQUIRED)
  include_directories(${sensor_msgs_INCLUDE_DIRS})

  find_package(ackermann_msgs REQUIRED)
  include_directories(${ackermann_msgs_INCLUDE_DIRS})

  find_package(tf REQUIRED)
  include_directories(${tf_INCLUDE_DIRS})
else()
  set(ROS_FOUND OFF)
  message(STATUS "Couldn't find roscpp so not building ROS-specific functionality. If you want ROS support, be sure to source the ROS setup.sh prior to building Drake.")
endif()

include_directories(thirdParty/spruce/include)
include_directories(thirdParty/cimg)

add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(core)
add_subdirectory(util)
add_subdirectory(solvers)
add_subdirectory(systems)
add_subdirectory(thirdParty/GetFullPath)
add_subdirectory(thirdParty/spruce)
add_subdirectory(thirdParty/tinyxml2)
add_subdirectory(examples)
add_subdirectory(bindings)

if(ROS_FOUND)
  add_subdirectory(ros)
endif()

# setup MATLAB pods-compliance
include(cmake/matlab_pods.cmake)
pods_configure_matlab_paths()

file(RELATIVE_PATH relpath ${CMAKE_INSTALL_PREFIX}/matlab ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS "Writing addpath_drake.m and rmpath_drake.m to ${CMAKE_INSTALL_PREFIX}/matlab")
file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_drake.m
  "function addpath_drake()\n"
  "  mfiledir = fileparts(which(mfilename));\n"
  "  wd = cd(fullfile(mfiledir,'${relpath}'));\n"
  "  addpath_drake();\n"
  "  cd(wd);\n"
  "end\n"
  "\n")

file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/rmpath_drake.m
  "function rmpath_drake()\n"
  "  mfiledir = fileparts(which(mfilename));\n"
  "  wd = cd(fullfile(mfiledir,'${relpath}'));\n"
  "  rmpath_drake();\n"
  "  cd(wd);\n")

find_program(avl avl PATHS ${CMAKE_INSTALL_DIR}/bin)
find_program(xfoil xfoil PATHS ${CMAKE_INSTALL_DIR}/bin)
find_program(ffmpeg ffmpeg)

# write some additional information to the CMake cache
set(lcm_java_classpath "${lcm-java_CLASSPATH}" CACHE FILEPATH "${lcm-java_CLASSPATH}" FORCE)

if(APPLE)
  set(DYLD_LIBRARY_PATH "$ENV{DYLD_LIBRARY_PATH}" CACHE STRING "Environment variable used to launch processes from Matlab")
  set(DYLD_FRAMEWORK_PATH "$ENV{DYLD_FRAMEWORK_PATH}" CACHE STRING "Environment variable used to launch processes from Matlab")
  mark_as_advanced(DYLD_LIBRARY_PATH)
  mark_as_advanced(DYLD_FRAMEWORK_PATH)
else()
  set(LD_LIBRARY_PATH "$ENV{LD_LIBRARY_PATH}" CACHE STRING "Environment variable used to launch processes from Matlab")
  mark_as_advanced(LD_LIBRARY_PATH)
endif()

if(MATLAB_FOUND)
  add_matlab_test(NAME "RigidBodyManipulatorMemoryTest"
    COMMAND "r = RigidBodyManipulator('Acrobot.urdf'), megaclear"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/Acrobot
    PROPERTIES TIMEOUT 300)
endif()

option(ENABLE_DOCUMENTATION "Enable build target for Doxygen and Sphinx documentation." ON)

if(ENABLE_DOCUMENTATION)
  add_subdirectory(doc)
endif()

if(IS_DIRECTORY doc/textbook)
  add_subdirectory(doc/textbook)
endif()
