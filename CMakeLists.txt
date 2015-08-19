cmake_minimum_required(VERSION 2.8.0)
project(drake-superbuild)

include(ExternalProject)

option(AUTO_UPDATE_EXTERNALS "external projects are updated to their tag revision on compile" ON)

# chinese menu of external projects
option(WITH_ALL_SUPPORTED_EXTERNALS "overrides the individual WITH_[external] options" OFF)
option(REMOVE_UNUSED_EXTERNALS "enable this to remove those projects from disk" OFF)

# ON by default:
option(WITH_EIGEN		"required c++ matrix library.  only disable if you have it already."    ON)
option(WITH_SPOTLESS	"polynomial optimization front-end for MATLAB"							ON)

# OFF by default:
option(WITH_GTK 		"precompiled gtk binaries/headers for Windows")
option(WITH_DIRECTOR	"vtk-based visualization tool and robot user interface")
option(WITH_SIGNALSCOPE	"live plotting tool for lcm messages")
option(WITH_IRIS		"fast approximate convex segmentation")
option(WITH_OCTOMAP 	"provides oct-tree data structures")
option(WITH_SNOPT 		"nonlinear optimization solver; requires access to RobotLocomotion/snopt-pod")
option(WITH_GUROBI 		"convex/integer optimization solver; free for academics (will prompt you for login bits)")
option(WITH_MOSEK 		"convex optimization solver; free for academics")
option(WITH_YALMIP 		"free optimization front-end for MATLAB")
option(WITH_GLOPTIPOLY 	"free global polynomial optimization tooblox")
option(WITH_BERTINI 	"solve polynomial equations; free but pod requires permissions (can't redistribute)")
option(WITH_SEDUMI		"semi-definite programming solver")
option(WITH_AVL			"use w/ XFOIL to compute aerodynamic coefficients for airfoils")
option(WITH_XFOIL		"use w/ AVL to compute aerodynamic coefficients for airfoils")
option(WITH_MESHCONVERTERS "uses vcglib to convert a few standard filetypes")

# system dependent defaults:
if (WIN32)
  option(WITH_LIBBOT 		"simple open-gl visualizer + lcmgl for director"	OFF)
  option(WITH_BULLET 		"used for collision detection" 					OFF)
  option(WITH_LCM 		"interprocess communications protocol for visualizers, etc" 			OFF)
else()
  option(WITH_LIBBOT 		"simple open-gl visualizer + lcmgl for director"	ON)
  option(WITH_BULLET 		"used for collision detection" 					ON)
  option(WITH_LCM 		"interprocess communications protocol for visualizers, etc" 			ON)
endif()

# list *compilation* dependencies, in alphabetical order by target (note: dependencies must come first in my foreach above)
set(lcm_dependencies gtk)
set(libbot_dependencies lcm)
set(director_dependencies lcm libbot)
set(signalscope_dependencies director)
set(drake_dependencies cmake eigen lcm libbot bullet octomap snopt gurobi)

# download information, in alphabetical order
set(avl_GIT_REPOSITORY https://github.com/RobotLocomotion/avl.git)
set(avl_GIT_TAG 9b927f90619460b29f7454c714369f65bc4536a1)
set(bertini_GIT_REPOSITORY https://github.com/RobotLocomotion/bertini.git)
set(bertini_GIT_TAG 3ae3e3ad3534acb4b6f7a4c3c22728b493beaa80)
set(bullet_GIT_REPOSITORY https://github.com/RobotLocomotion/bullet-pod.git)
set(bullet_GIT_TAG 6db7ae5e29f8b35ffcea384c53caa4bddcc6f986)
set(cmake_GIT_REPOSITORY https://github.com/RobotLocomotion/cmake.git)
set(cmake_GIT_TAG 451fdffdd0e7bfbfa53f1d5f50fad5196bb53bca)
set(cmake_GIT_CLONE_DIR "${PROJECT_SOURCE_DIR}/drake/cmake")
set(cmake_NO_BUILD TRUE)
set(director_GIT_REPOSITORY https://github.com/RobotLocomotion/director.git)
set(director_GIT_TAG 9c2bb9de70a0945d05eb0a4a7d13c0a91afa74a3)
set(director_SOURCE_DIR ${PROJECT_SOURCE_DIR}/externals/director/distro/pods/drake-distro)
set(signalscope_GIT_REPOSITORY https://github.com/mitdrc/signal-scope.git)
set(signalscope_GIT_TAG dc61a95daf31d25c730a075b5a8c9bba9eadafb3)
set(eigen_GIT_REPOSITORY https://github.com/RobotLocomotion/eigen-pod.git)
set(eigen_GIT_TAG fb56a4b8504eb115a7f257de4570c47c20c43397)
set(gtk_GIT_REPOSITORY https://github.com/RobotLocomotion/gtk-pod.git)
set(gtk_GIT_TAG dc9bb719ee771a2ae9c3a392796c42ccad5bca54)
set(gloptipoly_GIT_REPOSITORY https://github.com/RobotLocomotion/gloptipoly3.git)  # todo: rename that repo
set(gloptipoly_GIT_TAG a3547a6811bf76c4e18f00b6bcbc187c6a7fd3d8)
set(gurobi_GIT_REPOSITORY https://github.com/RobotLocomotion/gurobi.git)
set(gurobi_GIT_TAG beb0a84a1bcca1eb81a1b87d9fa140f2134d97e9)
set(iris_GIT_REPOSITORY https://github.com/rdeits/iris-distro.git)
set(iris_GIT_TAG 13468e0ea38e72ffae030adaa31e3015ea5f3ce0)
set(lcm_GIT_REPOSITORY https://github.com/RobotLocomotion/lcm-pod.git)
set(lcm_GIT_TAG 31e9a3e2dd056cac749fc2879e841434181180ad) # cmake branch 
set(libbot_GIT_REPOSITORY https://github.com/RobotLocomotion/libbot.git)
set(libbot_GIT_TAG c328b73f3b48008d04468031e26f48468a7e3cc0)
set(meshconverters_GIT_REPOSITORY https://github.com/RobotLocomotion/meshConverters.git)
set(meshconverters_GIT_TAG 34daf686ebdb7fa572cbe32af983dfe91a2af761)
set(mosek_GIT_REPOSITORY https://github.com/RobotLocomotion/mosek.git)
set(mosek_GIT_TAG a72594c4a7c8c5c026fd291148bcac76146634c2)
set(octomap_GIT_REPOSITORY https://github.com/RobotLocomotion/octomap-pod.git)
set(octomap_GIT_TAG 00b28215d19580f9e964bc5d126ce55a9253671a)
set(sedumi_GIT_REPOSITORY https://github.com/RobotLocomotion/sedumi.git)
set(sedumi_GIT_TAG afac8ea98f91007a3710d761efd514e10d0ad536)
set(snopt_GIT_REPOSITORY https://github.com/RobotLocomotion/snopt.git)
set(snopt_GIT_TAG d7abb603690f41970273643ebb6e173b0e6e452b)
set(spotless_GIT_REPOSITORY https://github.com/RobotLocomotion/spotless-pod.git)
set(spotless_GIT_TAG 565b720c25414a249d8b547183cf3ea6cf043211)
set(yalmip_GIT_REPOSITORY https://github.com/RobotLocomotion/yalmip.git)
set(yalmip_GIT_TAG c071fb7b7193491f8eefadba3bfff26160ad6cd4)
set(xfoil_GIT_REPOSITORY https://github.com/RobotLocomotion/xfoil.git)
set(xfoil_GIT_TAG fde0a9368dd451c93604051fc5704e120073800c)


find_program(MAKE_EXECUTABLE make)
if (NOT MAKE_EXECUTABLE)
	message(FATAL_ERROR "couldn't find gnu make")
endif()
if (${CMAKE_GENERATOR} STREQUAL "Unix Makefiles")
	set(PODS_MAKE_COMMAND "$(MAKE)")   # so we can pass through commandline arguments.
else()
	set(PODS_MAKE_COMMAND ${MAKE_EXECUTABLE})
endif()

# process optional projects
# note: keep drake in this loop in case externals depend on drake (e.g. the director might in the near future)
set(EXTERNAL_PROJECTS)
foreach(proj IN ITEMS cmake eigen gtk lcm libbot bullet iris spotless director signalscope octomap snopt gurobi mosek yalmip gloptipoly bertini sedumi avl xfoil meshconverters drake)
	string(TOUPPER ${proj} proj_upper)
	if (${proj} STREQUAL "drake" OR ${proj} STREQUAL "cmake" OR WITH_${proj_upper} OR WITH_ALL_SUPPORTED_EXTERNALS)
		list(APPEND EXTERNAL_PROJECTS ${proj})
	elseif(REMOVE_UNUSED_EXTERNALS AND IS_DIRECTORY ${proj})
		message(STATUS "removing unused project: ${proj}")
		if (NOT ${proj}_NO_BUILD)
			execute_process(COMMAND ${MAKE_EXECUTABLE} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE} clean
							WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${proj})
		endif()
		execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory "${PROJECT_SOURCE_DIR}/${proj}")
	endif()
endforeach()

# throw error for unsupported platform/project combinations
macro( unsupported )
	foreach(proj ${ARGN})
		list(FIND EXTERNAL_PROJECTS ${proj} find_result)
		if (NOT find_result EQUAL -1)
			if (WITH_ALL_SUPPORTED_EXTERNALS)
				list(REMOVE_ITEM EXTERNAL_PROJECTS ${proj})
			else()
				message(FATAL_ERROR ${proj} is not supported (yet) on this platform)
			endif()
		endif()
	endforeach()
endmacro()

if (WIN32)
	unsupported(libbot octomap gurobi mosek yalmip gloptipoly bertini sedumi avl xfoil iris director signalscope meshconverters)  # many of these could potentially work, but haven't been tried
	unsupported(bullet)  # drake has a silly c++11 compile error.  probably just for MSVC << 2013
	unsupported(lcm)  # just have to reliably add drake/build/lib to the path so that lcm-gen.exe can run when building drake.  note: move GTK back into ON by default for win32 only when I re-enable lcm
endif()

#enable_language(Fortran)
#if (NOT CMAKE_Fortran_COMPILER)
#   unsupported(avl xfoil)
#endif()


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
		if (NOT ${proj}_NO_BUILD)
			set(${proj}_BUILD_COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE})
		endif()

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
		ExternalProject_Add(${proj}
			SOURCE_DIR ${${proj}_SOURCE_DIR}
			DOWNLOAD_COMMAND ""
			UPDATE_COMMAND ""
			CONFIGURE_COMMAND ""
			BUILD_IN_SOURCE 1
			BUILD_COMMAND "${${proj}_BUILD_COMMAND}"
			INSTALL_COMMAND ""
			DEPENDS download-${proj} ${deps}
	#		BUILD_ALWAYS 1
			)
	else()
		message(STATUS "Preparing to build ${proj} with dependencies: ${deps}")

		ExternalProject_Add(${proj}
			SOURCE_DIR "${PROJECT_SOURCE_DIR}/drake"
			DOWNLOAD_COMMAND ""
			UPDATE_COMMAND ""
			CONFIGURE_COMMAND ""
			BUILD_IN_SOURCE 1
			BUILD_COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE}
			INSTALL_COMMAND ""
			DEPENDS ${deps}
	#		BUILD_ALWAYS 1
			)
	endif(NOT ${proj} STREQUAL "drake")


	# once we require cmake version >= 3.1, then we can zap this and just use the BUILD_ALWAYS flags above
 	ExternalProject_Add_Step(${proj} forceconfigure
 		COMMAND ${CMAKE_COMMAND} -E echo "Force configure of ${proj}"
 		DEPENDEES update
 		DEPENDERS configure
 		ALWAYS 1)

endforeach()


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
	if (NOT ${proj}_NO_BUILD)
		add_custom_target(clean-${proj}
			COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE} clean
			WORKING_DIRECTORY ${SOURCE_DIR})
		add_dependencies(clean-all clean-${proj})
	endif()
	list(APPEND PROJECT_LIST ${SOURCE_DIR})
endforeach()

string(REPLACE ";" " " PROJECT_LIST "${PROJECT_LIST}")
add_custom_target(list-project-dirs COMMAND echo "${PROJECT_LIST}")
