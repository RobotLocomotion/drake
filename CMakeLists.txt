cmake_minimum_required(VERSION 2.8.0)
project(drake-superbuild)

include(ExternalProject)

# menu of extra examples
option(EXAMPLES_LITTLEDOG "planning and control for a small quadruped robot" OFF)

## External dependencies
option(AUTO_UPDATE_EXTERNALS "external projects are updated to their tag revision on compile" ON)

# menu of external projects
option(WITH_ALL_PUBLIC_EXTERNALS "enable all externals available to public academic users" OFF)
option(WITH_ALL_SUPPORTED_EXTERNALS "enable all externals available for your platform (includes private git repositories)" OFF)
option(REMOVE_UNUSED_EXTERNALS "enable this to remove those projects from disk" OFF)

# ON by default:
option(WITH_EIGEN		"required c++ matrix library.  only disable if you have it already."    ON)

# OFF by default:
option(WITH_GTK 		"precompiled gtk binaries/headers for Windows")
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

# todo: make this on by default once it's tested:
option(WITH_SNOPT_PRECOMPILED "precompiled binaries only for snopt; the source requires a license (will be disabled if WITH_SNOPT=ON)"  OFF)


find_program(matlab matlab)
if (matlab)
  option(WITH_SPOTLESS	"polynomial optimization front-end for MATLAB"							ON)
else()
  option(WITH_SPOTLESS	"polynomial optimization front-end for MATLAB"							OFF)
endif()

# system dependent defaults:
if (WIN32)
  option(WITH_DIRECTOR	"vtk-based visualization tool and robot user interface"   OFF)
  option(WITH_LIBBOT 		"simple open-gl visualizer + lcmgl for director"	OFF)
  option(WITH_BULLET 		"used for collision detection" 					OFF)
  option(WITH_LCM 		"interprocess communications protocol for visualizers, etc" 			OFF)
else()
  option(WITH_DIRECTOR	"vtk-based visualization tool and robot user interface"   OFF)
  option(WITH_LIBBOT 		"simple open-gl visualizer + lcmgl for director"	ON)
  option(WITH_BULLET 		"used for collision detection" 					ON)
  option(WITH_LCM 		"interprocess communications protocol for visualizers, etc" 			ON)
endif()

# list *compilation* dependencies, in alphabetical order by target (note: dependencies must come first in my foreach above)
set(lcm_dependencies gtk)
set(libbot_dependencies lcm)
set(director_dependencies lcm libbot)
set(iris_dependencies eigen mosek)
set(signalscope_dependencies director)
set(drake_dependencies cmake eigen lcm libbot bullet octomap snopt gurobi)

# download information, in alphabetical order
set(avl_GIT_REPOSITORY https://github.com/RobotLocomotion/avl.git)
set(avl_GIT_TAG 9b927f90619460b29f7454c714369f65bc4536a1)
set(avl_IS_PUBLIC TRUE)
set(bertini_GIT_REPOSITORY https://github.com/RobotLocomotion/bertini.git)
set(bertini_GIT_TAG 3ae3e3ad3534acb4b6f7a4c3c22728b493beaa80)
set(bertini_IS_PUBLIC FALSE)
set(bullet_GIT_REPOSITORY https://github.com/RobotLocomotion/bullet-pod.git)
set(bullet_GIT_TAG 78c36cd116becf8417334f98bfcfd8d1300b03e0)
set(bullet_IS_PUBLIC TRUE)
set(cmake_GIT_REPOSITORY https://github.com/RobotLocomotion/cmake.git)
set(cmake_GIT_TAG 5ea95a6bb936c5fb8635b671a3355d86a882f5f7)
set(cmake_GIT_CLONE_DIR "${PROJECT_SOURCE_DIR}/drake/cmake")
set(cmake_BUILD_COMMAND "")
set(cmake_IS_PUBLIC TRUE)
set(director_GIT_REPOSITORY https://github.com/RobotLocomotion/director.git)
set(director_GIT_TAG 2bbd3cbbd4c06268b75263aca5056e9022c91bce)
set(director_SOURCE_DIR ${PROJECT_SOURCE_DIR}/externals/director/distro/pods/drake-distro)
set(director_IS_PUBLIC TRUE)
set(eigen_GIT_REPOSITORY https://github.com/RobotLocomotion/eigen-pod.git)
set(eigen_GIT_TAG 9fdb567778561415ca7032a64b8431484900798e)
set(eigen_IS_CMAKE_POD TRUE)
set(eigen_IS_PUBLIC TRUE)
set(gtk_GIT_REPOSITORY https://github.com/RobotLocomotion/gtk-pod.git)
set(gtk_GIT_TAG dc9bb719ee771a2ae9c3a392796c42ccad5bca54)
set(gtk_IS_PUBLIC TRUE)
set(gloptipoly_GIT_REPOSITORY https://github.com/RobotLocomotion/gloptipoly3.git)  # todo: rename that repo
set(gloptipoly_GIT_TAG c9e796e99c584ecf78317348c4162a2ed4121156)
set(gloptipoly_IS_PUBLIC FALSE)
set(gurobi_GIT_REPOSITORY https://github.com/RobotLocomotion/gurobi.git)
set(gurobi_GIT_TAG beb0a84a1bcca1eb81a1b87d9fa140f2134d97e9)
set(gurobi_IS_PUBLIC TRUE)
set(iris_GIT_REPOSITORY https://github.com/rdeits/iris-distro.git)
set(iris_GIT_TAG a17bdb4a20a8195363b629c47fd1b6251b44c95e)
set(iris_ADDITIONAL_BUILD_ARGS PATCH_COMMAND make configure-cdd-only)
set(iris_IS_PUBLIC TRUE)
set(lcm_GIT_REPOSITORY https://github.com/RobotLocomotion/lcm-pod.git)
set(lcm_GIT_TAG 4bb13061afafa126e58a1ac3345489425971ee04) # note: cmake branch
set(lcm_IS_CMAKE_POD TRUE)
set(lcm_SOURCE_DIR ${PROJECT_SOURCE_DIR}/externals/lcm/lcm-1.0.0)
set(lcm_IS_PUBLIC TRUE)
set(libbot_GIT_REPOSITORY https://github.com/RobotLocomotion/libbot.git)
set(libbot_GIT_TAG c328b73f3b48008d04468031e26f48468a7e3cc0)
set(libbot_IS_PUBLIC TRUE)
set(meshconverters_GIT_REPOSITORY https://github.com/RobotLocomotion/meshConverters.git)
set(meshconverters_GIT_TAG d50a79b80d74dd7a924daba7bd2011e03d9e66b4)
set(meshconverters_IS_PUBLIC TRUE)
set(mosek_GIT_REPOSITORY https://github.com/rdeits/mosek-pod.git)
set(mosek_GIT_TAG 1919b622df17b7584ac644406f360bd07cdf5fba)
set(mosek_IS_PUBLIC TRUE)
set(octomap_GIT_REPOSITORY https://github.com/RobotLocomotion/octomap-pod.git)
set(octomap_GIT_TAG 00b28215d19580f9e964bc5d126ce55a9253671a)
set(octomap_IS_PUBLIC TRUE)
set(sedumi_GIT_REPOSITORY https://github.com/RobotLocomotion/sedumi.git)
set(sedumi_GIT_TAG 8263577d4ab375524060c19369ccf410133bb9eb)
set(sedumi_IS_PUBLIC FALSE)
set(signalscope_GIT_REPOSITORY https://github.com/mitdrc/signal-scope.git)
set(signalscope_GIT_TAG bd891bb2a8284a969086b7f286462572e5698855)
set(signalscope_IS_PUBLIC TRUE)
set(snopt_GIT_REPOSITORY https://github.com/RobotLocomotion/snopt.git)
set(snopt_GIT_TAG e1b03db07d22ee11b2549d8a7c93466d263ce322)
set(snopt_IS_CMAKE_POD TRUE)
set(snopt_IS_PUBLIC FALSE)
set(spotless_GIT_REPOSITORY https://github.com/RobotLocomotion/spotless-pod.git)
set(spotless_GIT_TAG 28f97e3267b245c11a770abd57687906840a041d)
set(spotless_IS_CMAKE_POD TRUE)
set(spotless_IS_PUBLIC TRUE)
set(yalmip_GIT_REPOSITORY https://github.com/RobotLocomotion/yalmip.git)
set(yalmip_GIT_TAG c071fb7b7193491f8eefadba3bfff26160ad6cd4)
set(yalmip_IS_PUBLIC TRUE)
set(xfoil_GIT_REPOSITORY https://github.com/RobotLocomotion/xfoil.git)
set(xfoil_GIT_TAG fde0a9368dd451c93604051fc5704e120073800c)
set(xfoil_IS_PUBLIC TRUE)

# and now for the examples
set(LittleDog_GIT_REPOSITORY https://github.com/RobotLocomotion/LittleDog.git)
set(LittleDog_GIT_TAG a21aef548c33386243c40e315b1c70d83c122430)

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
foreach(proj IN ITEMS
		cmake
		eigen
		gtk
		lcm
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
		drake)
	string(TOUPPER ${proj} proj_upper)
	if (${proj} STREQUAL "drake" OR ${proj} STREQUAL "cmake" OR WITH_${proj_upper} OR WITH_ALL_SUPPORTED_EXTERNALS OR \(WITH_ALL_PUBLIC_EXTERNALS AND ${${proj}_IS_PUBLIC}\))
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
    if (NOT DEFINED ${proj}_CONFIGURE_COMMAND AND ${proj}_IS_CMAKE_POD)
      set(${proj}_CONFIGURE_COMMAND ${CMAKE_COMMAND} -E make_directory ${${proj}_SOURCE_DIR}/pod-build
        COMMAND ${CMAKE_COMMAND} -E chdir ${${proj}_SOURCE_DIR}/pod-build ${CMAKE_COMMAND} -G${CMAKE_GENERATOR} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} ..)
    endif()
		if (NOT DEFINED ${proj}_BUILD_COMMAND)
      if (${proj}_IS_CMAKE_POD)
        set(${proj}_BUILD_COMMAND ${CMAKE_COMMAND} -E chdir ${${proj}_SOURCE_DIR} ${CMAKE_COMMAND} --build pod-build --config ${CMAKE_BUILD_TYPE} --target install)
      else() # then use the PODS gnu make system
			   set(${proj}_BUILD_COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE})
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
		ExternalProject_Add(${proj}
			SOURCE_DIR ${${proj}_SOURCE_DIR}
#      BINARY_DIR ${${proj}_SOURCE_DIR}/pod-build
			DOWNLOAD_COMMAND ""
			UPDATE_COMMAND ""
			CONFIGURE_COMMAND "${${proj}_CONFIGURE_COMMAND}"
      BUILD_IN_SOURCE 1
			BUILD_COMMAND "${${proj}_BUILD_COMMAND}"
			INSTALL_COMMAND ""
			DEPENDS download-${proj} ${deps}
			${${proj}_ADDITIONAL_BUILD_ARGS}
			)
#    message(STATUS "${proj}_BUILD_COMMAND: ${${proj}_BUILD_COMMAND}")

	else()
		message(STATUS "Preparing to build ${proj} with dependencies: ${deps}")

		ExternalProject_Add(${proj}
			SOURCE_DIR "${PROJECT_SOURCE_DIR}/drake"
			DOWNLOAD_COMMAND ""
			UPDATE_COMMAND ""
      CONFIGURE_COMMAND ""
      BUILD_IN_SOURCE 1
      BUILD_COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE}
# note: missed one step in making drake the cmake (sub)project: it doesn't write the CTestTestfile so only runs the non-matlab unit tests.  Should be fixed easily enough.
#			CONFIGURE_COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/drake/pod-build
#        COMMAND ${CMAKE_COMMAND} -E chdir ${PROJECT_SOURCE_DIR}/drake/pod-build ${CMAKE_COMMAND} -G${CMAKE_GENERATOR} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} ..
#			BUILD_COMMAND ${CMAKE_COMMAND} --build ${PROJECT_SOURCE_DIR}/drake/pod-build --config ${CMAKE_BUILD_TYPE} --target install
			INSTALL_COMMAND ""
			DEPENDS ${deps}
			${${proj}_ADDITIONAL_BUILD_ARGS}
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
	if (NOT ${proj}_NO_BUILD)
		add_custom_target(clean-${proj}
			COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE} clean
			WORKING_DIRECTORY ${SOURCE_DIR})
		add_dependencies(clean-all clean-${proj})
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
if (NOT WIN32)  # just have to add the win libs to the zip to re-enable this support
  # todo: look for snopt_c
  if (snopt_c_FOUND OR WITH_SNOPT)
    set(WITH_SNOPT_PRECOMPILED OFF)
  endif()
  if (WITH_SNOPT_PRECOMPILED)
    message(STATUS "Preparing to install precompiled snopt")
    ExternalProject_Add(download-snopt-precompiled
      URL "http://drake002.csail.mit.edu/drake/drakeSnopt-2592a9e4e7666c6c3aafc49def9dc726f4942cea.tgz"  # todo: host this someplace better?
      SOURCE_DIR "${CMAKE_BINARY_DIR}/snopt-precompiled"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND cmake -E copy_directory ${CMAKE_BINARY_DIR}/snopt-precompiled/ ${PROJECT_SOURCE_DIR}/drake/solvers/
      INSTALL_COMMAND "")
    add_dependencies(download-all download-snopt-precompiled)
    add_dependencies(drake download-snopt-precompiled) # just in case: make sure any compiled drake version happens after precompiled install
  endif()
endif()
