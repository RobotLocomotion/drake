cmake_minimum_required(VERSION 2.8.10)
project(drake-superbuild)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
  message(FATAL_ERROR "GCC version must be at least 4.9")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.7)
  message(FATAL_ERROR "Clang version must be at least 3.7")
endif()

# PODs out-of-source build logic
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	if(POD_BUILD)
		find_file(_build_dir build PATHS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/.. ${PROJECT_SOURCE_DIR}/../.. ${PROJECT_SOURCE_DIR}/../../.. ${PROJECT_SOURCE_DIR}/../../../..)
		if (_build_dir)
			set(CMAKE_INSTALL_PREFIX "${_build_dir}" CACHE STRING
				"Prefix for installation of sub-packages (note: required during build!)" FORCE)
		else()
			execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/build)
			set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/build
			CACHE STRING "install prefix" FORCE)
		endif()
	else()
		set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
			"Prefix for installation of sub-packages (note: required during build!)" FORCE)
	endif()
endif()
message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

include(ExternalProject)
function(drake_forcebuild proj)
  if(CMAKE_CONFIGURATION_TYPES)
    set(build_stamp_file "${CMAKE_CURRENT_BINARY_DIR}/${proj}-prefix/src/${proj}-stamp/${CMAKE_CFG_INTDIR}/${proj}-build")
  else()
    set(build_stamp_file "${CMAKE_CURRENT_BINARY_DIR}/${proj}-prefix/src/${proj}-stamp/${proj}-build")
  endif()
  ExternalProject_Add_Step(${proj} forcebuild
    COMMAND ${CMAKE_COMMAND} -E remove ${build_stamp_file}
    COMMENT "Forcing build step for '${proj}'"
    DEPENDEES build
    ALWAYS 1
    )
endfunction()

# Require Java at the super-build level so libbot cannot configure without finding Java
find_package(Java REQUIRED)
include(UseJava)

# menu of extra examples
option(EXAMPLES_LITTLEDOG "planning and control for a small quadruped robot" OFF)

## External dependencies
option(AUTO_UPDATE_EXTERNALS "external projects are updated to their tag revision on compile" ON)

# Choose your python (major) version
option(WITH_PYTHON_3 "Force Drake to use python 3 instead of python 2" OFF)

# menu of external projects
option(WITH_ALL_PUBLIC_EXTERNALS "enable all externals available to public academic users" OFF)
option(WITH_ALL_SUPPORTED_EXTERNALS "enable all externals available for your platform (includes private git repositories)" OFF)
option(REMOVE_UNUSED_EXTERNALS "enable this to remove those projects from disk" OFF)

# ON by default:
option(WITH_EIGEN		"required c++ matrix library.  only disable if you have it already."    ON)
option(WITH_GOOGLETEST "required c++ unit test library.  only disable if you have it already." ON)
option(WITH_SWIGMAKE    "helper tools to build python & MATLAB wrappers for C++ libraries with Eigen" ON)
option(WITH_BULLET 		"used for collision detection" 	ON)
option(WITH_LCM 		"interprocess communications protocol for visualizers, etc" ON)
option(WITH_BOT_CORE_LCMTYPES "required LCM types library. only disable if you have it already." ON)
if (WIN32)
	option(WITH_GTK 		"precompiled gtk binaries/headers for Windows"  ON)  # needed for lcm on windows
else()
	option(WITH_DIRECTOR	"vtk-based visualization tool and robot user interface"   ON) # not win32 yet.  it builds on windows, but requires manually installation of vtk, etc.  perhaps make a precompiled director pod (a bit like snopt)
	option(WITH_LIBBOT 		"simple open-gl visualizer + lcmgl for director"	ON) # there is hope, but i spent a long time cleaning up c/c++ language errors trying to make msvc happy.. and still had a long way to go.
	option(WITH_SWIG_MATLAB "A version of SWIG with MATLAB support" ON)
endif()

include(CMakeDependentOption)
option(DISABLE_MATLAB "Don't use MATLAB even if it is present." OFF)
if (DISABLE_MATLAB)
	message(STATUS "MATLAB is disabled.")
else()
	find_program(matlab matlab)
	if (matlab)
		message(STATUS "Found MATLAB at " ${matlab})
	else ()
		message(STATUS "Looked for MATLAB but could not find it.")
	endif(matlab)
endif (DISABLE_MATLAB)

# If DISABLE_MATLAB is set or matlab is not found, WITH_SPOTLESS is hidden and default OFF.
# Otherwise, WITH_SPOTLESS is visible and default ON.
CMAKE_DEPENDENT_OPTION(WITH_SPOTLESS "polynomial optimization front-end for MATLAB" ON "NOT DISABLE_MATLAB;matlab" OFF)

option(WITH_YAML_CPP "library for reading and writing yaml configuration files" ON)

option(WITH_SNOPT_PRECOMPILED "precompiled binaries only for snopt; the source requires a license (will be disabled if WITH_SNOPT=ON)"  ON)

# OFF by default:
option(WITH_SNOPT 		"nonlinear optimization solver; requires access to RobotLocomotion/snopt-pod")

if (NOT WIN32) # many of these might work on win32 with little or no work... they just haven't been tried
	option(WITH_SIGNALSCOPE	"live plotting tool for lcm messages")
	option(WITH_IRIS		"fast approximate convex segmentation")
	option(WITH_OCTOMAP 	"provides oct-tree data structures")
	option(WITH_GUROBI 		"convex/integer optimization solver; free for academics (will prompt you for login bits)")
	option(WITH_MOSEK 		"convex optimization solver; free for academics")
	option(WITH_NLOPT 		"nonlinear optimization solver"  ON)
	option(WITH_YALMIP 		"free optimization front-end for MATLAB")
	option(WITH_GLOPTIPOLY 	"free global polynomial optimization tooblox")
	option(WITH_BERTINI 	"solve polynomial equations; free but pod requires permissions (can't redistribute)")
	option(WITH_SEDUMI		"semi-definite programming solver")
	option(WITH_TEXTBOOK "pull in the Underactuated Robotics textbook and its examples")  # almost works on windows.  the update step call to git was complaining about local modifications on drake003
	#  include(CMakeDetermineFortranCompiler) # this didn't do the trick, and enable_language(Fortran OPTIONAL) simply fails if not found (optional is a placeholder option that doesn't work yet)
	#  if (CMAKE_Fortran_COMPILER)
	option(WITH_AVL			"use w/ AVL to compute aerodynamic coefficients for airfoils")
	option(WITH_XFOIL		"use w/ XFOIL to compute aerodynamic coefficients for airfoils")
	#  endif()
	option(WITH_MESHCONVERTERS "uses vcglib to convert a few standard filetypes")
endif()

if (WITH_TEXTBOOK OR WITH_ALL_SUPPORTED_EXTERNALS OR WITH_ALL_PUBLIC_EXTERNALS)
#  find_package(PythonInterp)
  find_program(PYTHON_EXECUTABLE NAMES python2.7 python2.6) # PythonInterp finds a symlink on cygwin, which then fails in the execute process below
	if (NOT PYTHON_EXECUTABLE)
		message(FATAL_ERROR "could not find python, which is required for the the textbook examples")
	endif()
endif()

# list *compilation* dependencies, in alphabetical order by target (note: dependencies must come first in my foreach above)
set(lcm_dependencies gtk)
set(libbot_dependencies lcm)
set(bot_core_lcmtypes_dependencies lcm)
set(director_dependencies lcm bot_core_lcmtypes libbot)
set(iris_dependencies eigen mosek)
set(signalscope_dependencies director)
set(drake_dependencies cmake eigen googletest lcm bot_core_lcmtypes libbot bullet octomap snopt gurobi swig_matlab swigmake yaml_cpp)

# download information, in alphabetical order
set(avl_GIT_REPOSITORY https://github.com/RobotLocomotion/avl.git)
set(avl_GIT_TAG 9b927f90619460b29f7454c714369f65bc4536a1)
set(avl_IS_PUBLIC TRUE)
set(bertini_GIT_REPOSITORY git@github.com:RobotLocomotion/bertini.git)
set(bertini_GIT_TAG 3ae3e3ad3534acb4b6f7a4c3c22728b493beaa80)
set(bertini_IS_PUBLIC FALSE)
set(bullet_GIT_REPOSITORY https://github.com/RobotLocomotion/bullet-pod.git)
set(bullet_GIT_TAG f7cfcf6a24977b57dc832593743c770548799e45)
set(bullet_IS_PUBLIC TRUE)
set(bullet_IS_CMAKE_POD TRUE)
set(cmake_GIT_REPOSITORY https://github.com/RobotLocomotion/cmake.git)
set(cmake_GIT_TAG abe1a22d0bb767423de4e3bd1123cc2deae1c82a)
set(cmake_GIT_CLONE_DIR "${PROJECT_SOURCE_DIR}/drake/cmake")
set(cmake_NO_BUILD TRUE)
set(cmake_IS_PUBLIC TRUE)
set(director_GIT_REPOSITORY https://github.com/RobotLocomotion/director.git)
set(director_GIT_TAG cfc7702bf9278707bca82d0393bba8611513ea15)
set(director_SOURCE_DIR ${PROJECT_SOURCE_DIR}/externals/director/distro/pods/drake-distro)
set(director_IS_PUBLIC TRUE)
set(drake_java_args -DJava_JAVA_EXECUTABLE=${Java_JAVA_EXECUTABLE} -DJava_JAVAC_EXECUTABLE=${Java_JAVAC_EXECUTABLE} -DJava_JAVAH_EXECUTABLE=${Java_JAVAH_EXECUTABLE} -DJava_VERSION_STRING=${Java_VERSION_STRING})
set(drake_ADDITIONAL_CMAKE_CONFIGURE_ARGS ${drake_java_args} -DWITH_PYTHON_3=${WITH_PYTHON_3} -DDISABLE_MATLAB=${DISABLE_MATLAB})
set(eigen_GIT_REPOSITORY https://github.com/RobotLocomotion/eigen-pod.git)
set(eigen_GIT_TAG 20a633e8f0a285edf9dcc5a06cc9e80774920a7f)
set(eigen_IS_CMAKE_POD TRUE)
set(eigen_IS_PUBLIC TRUE)
set(googletest_GIT_REPOSITORY https://github.com/google/googletest.git)
set(googletest_GIT_TAG ff07a5de0e81580547f1685e101194ed1a4fcd56)
set(googletest_IS_CMAKE_POD TRUE)
set(googletest_IS_PUBLIC TRUE)
set(googletest_ADDITIONAL_CMAKE_CONFIGURE_ARGS -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_NAME_DIR=${CMAKE_INSTALL_PREFIX}/lib -DGTEST_CREATE_SHARED_LIBRARY=1) # The "-DGTEST_CREATE_SHARED_LIBRARY=1" is needed to support parameterized Google tests on Windows.
set(gtk_GIT_REPOSITORY https://github.com/RobotLocomotion/gtk-pod.git)
set(gtk_GIT_TAG 4730a7d3274a35280f1c2c6337eb182935c87fc2)
set(gtk_IS_CMAKE_POD TRUE)
set(gtk_IS_PUBLIC TRUE)
set(gloptipoly_GIT_REPOSITORY git@github.com:RobotLocomotion/gloptipoly3.git)  # todo: rename that repo
set(gloptipoly_GIT_TAG c9e796e99c584ecf78317348c4162a2ed4121156)
set(gloptipoly_IS_PUBLIC FALSE)
set(gurobi_GIT_REPOSITORY https://github.com/RobotLocomotion/gurobi.git)
set(gurobi_GIT_TAG fbbdbbc0152ddb37df209d221aff37448b8ff29f)
set(gurobi_IS_PUBLIC FALSE)
set(iris_GIT_REPOSITORY https://github.com/rdeits/iris-distro.git)
set(iris_GIT_TAG 1533f20cff57504a46773f01e1730f11931d0ea6)
set(iris_ADDITIONAL_BUILD_ARGS PATCH_COMMAND make configure-cdd-only)
set(iris_IS_PUBLIC TRUE)
set(lcm_GIT_REPOSITORY https://github.com/RobotLocomotion/lcm-pod.git)
set(lcm_GIT_TAG a0181fc5e86677af9bfbc21622df3d074b612c95) # note: cmake branch
set(lcm_IS_CMAKE_POD TRUE)
set(lcm_IS_PUBLIC TRUE)
set(libbot_GIT_REPOSITORY https://github.com/RobotLocomotion/libbot.git)
set(libbot_GIT_TAG cc8d228b50847c4c55e6963b8ee95c237287547f)
set(libbot_IS_CMAKE_POD TRUE)
set(libbot_IS_PUBLIC TRUE)
set(bot_core_lcmtypes_GIT_REPOSITORY  https://github.com/jhoare/bot_core_lcmtypes)
set(bot_core_lcmtypes_GIT_TAG fc70d061005f52000cf8af7658c34e86bc2bac68)
set(bot_core_lcmtypes_IS_CMAKE_POD TRUE)
set(bot_core_lcmtypes_IS_PUBLIC TRUE)
set(meshconverters_GIT_REPOSITORY https://github.com/RobotLocomotion/meshConverters.git)
set(meshconverters_GIT_TAG d50a79b80d74dd7a924daba7bd2011e03d9e66b4)
set(meshconverters_IS_PUBLIC TRUE)
set(mosek_GIT_REPOSITORY https://github.com/rdeits/mosek-pod.git)
set(mosek_GIT_TAG 1919b622df17b7584ac644406f360bd07cdf5fba)
set(mosek_IS_PUBLIC TRUE)
set(nlopt_GIT_REPOSITORY https://github.com/stevengj/nlopt.git)
set(nlopt_GIT_TAG 78e165f9defbac487e915219b7a0866fd25e4c62)
set(nlopt_BUILD_COMMAND ./autogen.sh --no-configure && ./configure --without-matlab --without-python --without-octave --without-guile --enable-maintainer-mode --enable-shared --prefix=${CMAKE_INSTALL_PREFIX} --includedir=${CMAKE_INSTALL_PREFIX}/include/nlopt && make install)
set(nlopt_IS_PUBLIC TRUE)
set(octomap_GIT_REPOSITORY https://github.com/RobotLocomotion/octomap-pod.git)
set(octomap_GIT_TAG 00b28215d19580f9e964bc5d126ce55a9253671a)
set(octomap_IS_PUBLIC TRUE)
set(sedumi_GIT_REPOSITORY git@github.com:RobotLocomotion/sedumi.git)
set(sedumi_GIT_TAG 8263577d4ab375524060c19369ccf410133bb9eb)
set(sedumi_IS_PUBLIC FALSE)
set(signalscope_GIT_REPOSITORY https://github.com/mitdrc/signal-scope.git)
set(signalscope_GIT_TAG f22243169381f99f29e534931014557b50b4af95)
set(signalscope_IS_PUBLIC TRUE)
set(snopt_GIT_REPOSITORY git@github.com:RobotLocomotion/snopt.git)
set(snopt_GIT_TAG 95d908275156f2665ef3941f08cb89c767480a6e)
set(snopt_IS_CMAKE_POD TRUE)
set(snopt_IS_PUBLIC FALSE)
set(spotless_GIT_REPOSITORY https://github.com/RobotLocomotion/spotless-pod.git)
set(spotless_GIT_TAG 86275d25eecebe5e85d33e1074cdb274a1b32d49)
set(spotless_IS_CMAKE_POD TRUE)
set(spotless_IS_PUBLIC TRUE)
set(swigmake_GIT_REPOSITORY https://github.com/rdeits/swigmake.git)
set(swigmake_GIT_TAG 7e4fcbeb46c5fc7b1efb651b4365ba8b777ef184)
set(swigmake_IS_PUBLIC TRUE)
set(swigmake_IS_CMAKE_POD TRUE)
set(swigmake_NO_CLEAN TRUE)
set(swig_matlab_GIT_REPOSITORY https://github.com/rdeits/swig-matlab-pod.git)
set(swig_matlab_GIT_TAG a763cac0a0862849f2f33d295cc398014531acf4)
set(swig_matlab_IS_PUBLIC TRUE)
set(swig_matlab_IS_CMAKE_POD TRUE)
set(textbook_GIT_REPOSITORY https://github.com/RussTedrake/underactuated.git)
set(textbook_GIT_TAG master)
set(textbook_GIT_CLONE_DIR "${PROJECT_SOURCE_DIR}/drake/doc/textbook")
set(textbook_BUILD_COMMAND ${PYTHON_EXECUTABLE} extract_examples.py underactuated.html ./examples)
set(textbook_IS_PUBLIC TRUE)
set(yalmip_GIT_REPOSITORY https://github.com/RobotLocomotion/yalmip.git)
set(yalmip_GIT_TAG c071fb7b7193491f8eefadba3bfff26160ad6cd4)
set(yalmip_IS_PUBLIC TRUE)
set(yaml_cpp_GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git)
set(yaml_cpp_GIT_TAG 57805dfd6a741e55477ea8d4d5b3b6f0272d1f0e)
set(yaml_cpp_IS_PUBLIC TRUE)
set(yaml_cpp_IS_CMAKE_POD TRUE)
if (APPLE OR WIN32)
	set(yaml_cpp_ADDITIONAL_CMAKE_CONFIGURE_ARGS -DBUILD_SHARED_LIBS=OFF)
else()
	set(yaml_cpp_ADDITIONAL_CMAKE_CONFIGURE_ARGS -DBUILD_SHARED_LIBS=ON)
endif()
set(xfoil_GIT_REPOSITORY https://github.com/RobotLocomotion/xfoil.git)
set(xfoil_GIT_TAG fde0a9368dd451c93604051fc5704e120073800c)
set(xfoil_IS_PUBLIC TRUE)

# and now for the examples
set(LittleDog_GIT_REPOSITORY https://github.com/RobotLocomotion/LittleDog.git)
set(LittleDog_GIT_TAG b1e2740a183afdcb8c27bbdee461b8ada1c3550e)

find_program(MAKE_EXECUTABLE make)
if (NOT MAKE_EXECUTABLE)
	message(WARNING "couldn't find gnu make")
endif()
if (${CMAKE_GENERATOR} STREQUAL "Unix Makefiles")
	set(PODS_MAKE_COMMAND "$(MAKE)")   # so we can pass through commandline arguments.
else()
	set(PODS_MAKE_COMMAND ${MAKE_EXECUTABLE})
endif()

# process optional projects
# note: keep drake in this loop in case externals depend on drake (e.g. the director might in the near future)
set(EXTERNAL_PROJECTS)
foreach(proj IN ITEMS
		cmake
		textbook
		eigen
		googletest
		gtk
		lcm
		bot_core_lcmtypes
		libbot
		bullet
		spotless
		director
		signalscope
		octomap
		snopt
		gurobi
		mosek
		iris
		yalmip
		gloptipoly
		bertini
		sedumi
		avl
		xfoil
		meshconverters
		swigmake
		swig_matlab
		yaml_cpp
		nlopt
		drake)
	string(TOUPPER ${proj} proj_upper)
	if ( ${proj} STREQUAL "drake" OR
			${proj} STREQUAL "cmake" OR
			WITH_${proj_upper} OR
	(WITH_ALL_SUPPORTED_EXTERNALS AND DEFINED WITH_${proj_upper}) OR
	(WITH_ALL_PUBLIC_EXTERNALS AND ${proj}_IS_PUBLIC AND DEFINED WITH_${proj_upper}) )
		list(APPEND EXTERNAL_PROJECTS ${proj})
	elseif(REMOVE_UNUSED_EXTERNALS AND IS_DIRECTORY ${proj})
		message(STATUS "removing unused project: ${proj}")
		if (NOT ${proj}_NO_BUILD)
			execute_process(COMMAND ${MAKE_EXECUTABLE} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=$<CONFIGURATION> clean
					WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${proj})
		endif()
		execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory "${PROJECT_SOURCE_DIR}/${proj}")
	endif()
endforeach()

set(COMMON_CMAKE_ARGS
	CMAKE_GENERATOR ${CMAKE_GENERATOR}
	CMAKE_ARGS
	-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
	-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
	-DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}
	-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
	-DCMAKE_EXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}
	-DCMAKE_MODULE_LINKER_FLAGS=${CMAKE_MODULE_LINKER_FLAGS}
	-DCMAKE_SHARED_LINKER_FLAGS=${CMAKE_SHARED_LINKER_FLAGS}
	-DCMAKE_STATIC_LINKER_FLAGS=${CMAKE_STATIC_LINKER_FLAGS}
)

set(EXTERNAL_SOURCE_DIRS)
foreach (proj ${EXTERNAL_PROJECTS})
	set(deps)
	foreach(dep ${${proj}_dependencies})
		list(FIND EXTERNAL_PROJECTS ${dep} find_result)
		if (${dep} STREQUAL "drake" OR NOT find_result EQUAL -1)
			list(APPEND deps ${dep})
		endif()
	endforeach()

	if (NOT ${proj} STREQUAL "drake")
		if (NOT ${proj}_GIT_CLONE_DIR)
			set(${proj}_GIT_CLONE_DIR "${PROJECT_SOURCE_DIR}/externals/${proj}")
		endif()
		if (NOT ${proj}_SOURCE_DIR)
			set(${proj}_SOURCE_DIR ${${proj}_GIT_CLONE_DIR})
		endif()
		if (NOT ${proj}_BINARY_DIR)
			if(POD_BUILD)
				set(${proj}_BINARY_DIR "${${proj}_SOURCE_DIR}/pod-build")
			else()
				set(${proj}_BINARY_DIR "${PROJECT_BINARY_DIR}/externals/${proj}")
			endif()
		endif()
		set(EXTERNAL_SOURCE_DIRS ${EXTERNAL_SOURCE_DIRS} ${${proj}_SOURCE_DIR} "\n")

		message(STATUS "Preparing to build ${proj} with dependencies: ${deps}")

		# separate download target so I can make the download-all custom command as recommended in:
		#   http://comments.gmane.org/gmane.comp.programming.tools.cmake.user/53002
		if (AUTO_UPDATE_EXTERNALS)
			ExternalProject_Add(download-${proj}
					DOWNLOAD_DIR ${${proj}_GIT_CLONE_DIR}
					SOURCE_DIR ${${proj}_GIT_CLONE_DIR}
					GIT_REPOSITORY ${${proj}_GIT_REPOSITORY}
					GIT_TAG ${${proj}_GIT_TAG}
					CONFIGURE_COMMAND ""
					BUILD_COMMAND ""
					INSTALL_COMMAND ""
					)
		else()
			ExternalProject_Add(download-${proj}
					DOWNLOAD_DIR ${${proj}_GIT_CLONE_DIR}
					SOURCE_DIR ${${proj}_GIT_CLONE_DIR}
					GIT_REPOSITORY ${${proj}_GIT_REPOSITORY}
					GIT_TAG ${${proj}_GIT_TAG}
					UPDATE_COMMAND ""
					CONFIGURE_COMMAND ""
					BUILD_COMMAND ""
					INSTALL_COMMAND ""
					)
		endif()

		# now the actual project
		if (${proj}_IS_CMAKE_POD)
			set(PODS_VERBOSE_MAKEFILE "")
			if (CMAKE_VERBOSE_MAKEFILE)
			    set(PODS_VERBOSE_MAKEFILE "-DCMAKE_VERBOSE_MAKEFILE=ON")
			endif()
			ExternalProject_Add(${proj}
					SOURCE_DIR ${${proj}_SOURCE_DIR}
					DOWNLOAD_COMMAND ""
					UPDATE_COMMAND ""
					BINARY_DIR ${${proj}_BINARY_DIR}
					DEPENDS download-${proj} ${deps}
					${COMMON_CMAKE_ARGS}
					${PODS_VERBOSE_MAKEFILE}
					${${proj}_ADDITIONAL_CMAKE_CONFIGURE_ARGS}
					${${proj}_ADDITIONAL_BUILD_ARGS}
					)
			drake_forcebuild(${proj})
		else() # not a CMake POD
			if (NOT DEFINED ${proj}_BUILD_COMMAND)
				if(${proj}_NO_BUILD)
					set(${proj}_BUILD_COMMAND "")
				else() # then use the PODS gnu make system
					set(PODS_VERBOSE_MAKEFILE "")
					if (CMAKE_VERBOSE_MAKEFILE)
					    set(PODS_VERBOSE_MAKEFILE "V=1 VERBOSE=1")
					endif()
					set(${proj}_BUILD_COMMAND ${PODS_MAKE_COMMAND} ${PODS_VERBOSE_MAKEFILE} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=$<CONFIGURATION> )
				endif()
			endif()

			ExternalProject_Add(${proj}
					SOURCE_DIR ${${proj}_SOURCE_DIR}
					#      BINARY_DIR ${${proj}_SOURCE_DIR}/pod-build
					DOWNLOAD_COMMAND ""
					UPDATE_COMMAND ""
					CONFIGURE_COMMAND ""
					BUILD_COMMAND "${${proj}_BUILD_COMMAND}"
					BUILD_IN_SOURCE 1
					INSTALL_COMMAND ""
					DEPENDS download-${proj} ${deps}
					${${proj}_ADDITIONAL_BUILD_ARGS}
					)
			if (${proj}_BUILD_COMMAND)
				drake_forcebuild(${proj})
			endif()
			#    message(STATUS "${proj}_BUILD_COMMAND: ${${proj}_BUILD_COMMAND}")
		endif()

	else()
		message(STATUS "Preparing to build ${proj} with dependencies: ${deps}")
		set(PODS_VERBOSE_MAKEFILE "")
		if (CMAKE_VERBOSE_MAKEFILE)
		    set(PODS_VERBOSE_MAKEFILE "-DCMAKE_VERBOSE_MAKEFILE=ON")
		endif()
		if(POD_BUILD)
			set(drake_BINARY_DIR "${PROJECT_SOURCE_DIR}/drake/pod-build")
		else()
			set(drake_BINARY_DIR "${PROJECT_BINARY_DIR}/drake")
		endif()
		ExternalProject_Add(${proj}
				SOURCE_DIR "${PROJECT_SOURCE_DIR}/drake"
				DOWNLOAD_COMMAND ""
				UPDATE_COMMAND ""
				BINARY_DIR ${drake_BINARY_DIR}
				DEPENDS ${deps}
				${COMMON_CMAKE_ARGS}
				${PODS_VERBOSE_MAKEFILE}
				${${proj}_ADDITIONAL_CMAKE_CONFIGURE_ARGS}
				${${proj}_ADDITIONAL_BUILD_ARGS}
				)
		drake_forcebuild(${proj})
	endif(NOT ${proj} STREQUAL "drake")

endforeach()
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/drake_external_source_dirs.txt ${EXTERNAL_SOURCE_DIRS})

# todo: add a custom target for release_filelist

add_custom_target(download-all)
add_custom_target(clean-all)
add_custom_target(status)
set(PROJECT_LIST)
foreach (proj ${EXTERNAL_PROJECTS})
	if (NOT ${proj} STREQUAL "drake")
		add_dependencies(download-all download-${proj})
		ExternalProject_Get_Property(download-${proj} SOURCE_DIR)
		add_custom_target(status-${proj}
				COMMAND ${GIT_EXECUTABLE} status
				WORKING_DIRECTORY ${SOURCE_DIR})
		add_dependencies(status status-${proj})
	endif()

  ExternalProject_Get_Property(${proj} SOURCE_DIR)
  # Handle pods-style 'make clean' cleaning of external projects/pods
  if (NOT ${proj}_NO_BUILD)
    if (${proj}_IS_CMAKE_POD)
      ExternalProject_Get_Property(${proj} BINARY_DIR)
      # if _NO_CLEAN is set, don't attempt to run `make clean`, just wipe the build folder
      if (${proj}_NO_CLEAN)
        add_custom_target(clean-${proj}
          COMMAND ${CMAKE_COMMAND} -E remove_directory ${BINARY_DIR})
      else()
        add_custom_target(clean-${proj}
          COMMAND ${CMAKE_COMMAND} --build ${BINARY_DIR} --target clean
          COMMAND ${CMAKE_COMMAND} -E remove_directory ${BINARY_DIR})
      endif()
      add_dependencies(clean-all clean-${proj})
    else()
      add_custom_target(clean-${proj}
          COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=$<CONFIGURATION> clean
          WORKING_DIRECTORY ${SOURCE_DIR})
      add_dependencies(clean-all clean-${proj})
    endif()
  endif()
	list(APPEND PROJECT_LIST ${SOURCE_DIR})
endforeach()

# process optional examples
foreach (example IN ITEMS
		LittleDog)
	string(TOUPPER ${example} example_upper)
	if (EXAMPLES_${example_upper})
		message(STATUS "Installation will include extra example: ${example}")
		ExternalProject_Add(download-${example}
				DOWNLOAD_DIR ${PROJECT_SOURCE_DIR}/drake/examples/${example}
				SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake/examples/${example}
				GIT_REPOSITORY ${${example}_GIT_REPOSITORY}
				GIT_TAG ${${example}_GIT_TAG}
				CONFIGURE_COMMAND ""
				BUILD_COMMAND ""
				INSTALL_COMMAND ""
				)
		add_dependencies(drake download-${example})
		add_dependencies(download-all download-${example})
	endif()
endforeach()

string(REPLACE ";" " " PROJECT_LIST "${PROJECT_LIST}")
add_custom_target(list-project-dirs COMMAND echo "${PROJECT_LIST}")

## grab and install precompiled snopt

# todo: look for snopt_c
if (snopt_c_FOUND OR WITH_SNOPT)
  set(WITH_SNOPT_PRECOMPILED OFF)
endif()
if (WITH_SNOPT_PRECOMPILED)
  message(STATUS "Preparing to install precompiled snopt")
  ExternalProject_Add(download-snopt-precompiled
		URL "https://s3.amazonaws.com/drake-provisioning/drakeSnopt.zip"
		URL_MD5 "7b36168cba2fb9a56b2fd6117427fc4a"
		SOURCE_DIR "${CMAKE_BINARY_DIR}/snopt-precompiled"
		CONFIGURE_COMMAND ""
		BUILD_COMMAND cmake -E copy_directory ${CMAKE_BINARY_DIR}/snopt-precompiled/ ${PROJECT_SOURCE_DIR}/drake/solvers/
		INSTALL_COMMAND "")
  add_dependencies(download-all download-snopt-precompiled)
  add_dependencies(drake download-snopt-precompiled) # just in case: make sure any compiled drake version happens after precompiled install
endif()
