
if (CMAKE_COMPILER_IS_GNUCC)
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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 if (APPLE)  # this was a step towards getting things to work with
#   clang on mac, but ultimately we didn't get there...   (but I would
#   be worried about sharing pointers between objects compiled against
#   different c++ libs, so removing it)
#    # http://stackoverflow.com/questions/13445742/apple-and-shared-ptr
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")  
  endif()
endif()

find_package( Boost COMPONENTS filesystem system regex)

include_directories (${CMAKE_SOURCE_DIR}/util )

if (Boost_FOUND)
  include_directories ( ${Boost_INCLUDE_DIR} tinyxml )
  link_directories( ${Boost_LIBRARY_DIRS} )
endif()

pods_find_pkg_config(bot2-vis)
if (bot2-vis_FOUND)
  add_definitions( -DBOT_VIS_SUPPORT )   
endif()

if (eigen3_FOUND)
  # todo: check eigen version.  3.1.0 didn't work for clang.  3.2.0 did.

  add_library(drakeRBM SHARED RigidBodyManipulator.cpp RigidBody.cpp)
  target_link_libraries(drakeRBM drakeCollision)

  add_subdirectory(collision)

  if (Boost_FOUND)
    add_library(URDFRigidBodyManipulator URDFRigidBodyManipulator.cpp tinyxml/tinyxml.cpp tinyxml/tinyxmlparser.cpp tinyxml/tinyxmlerror.cpp)
    target_link_libraries(URDFRigidBodyManipulator drakeRBM urdf_interface ${Boost_LIBRARIES})

    if (bot2-vis_FOUND)
	   pods_use_pkg_config_packages(URDFRigidBodyManipulator bot2-vis)
    endif()

    add_subdirectory(urdf_interface)
    add_subdirectory(viewer)
  endif(Boost_FOUND)

  macro(add_rbm_mex)
    # todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
#    add_mex(${ARGV} ${ARGV}.cpp $<TARGET_OBJECTS:drakeUtil>)
#    target_link_libraries(${ARGV} drakeRBM)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeUtil)
  endmacro()

  add_rbm_mex(constructModelmex)
  add_rbm_mex(deleteModelmex)
  add_rbm_mex(HandCmex)
  add_rbm_mex(doKinematicsmex)
  add_rbm_mex(forwardKinmex)
  add_rbm_mex(bodyKinmex)
  add_rbm_mex(collisionmex)
  add_rbm_mex(getCMMmex)

  pods_find_pkg_config(gurobi)
  if (gurobi_FOUND)
    add_rbm_mex(approximateIKmex)
    pods_use_pkg_config_packages(approximateIKmex gurobi)
  endif()

  #	approximateIKEIQPmex.cpp
  #	collisionmex.cpp 

  macro(add_planar_rbm_mex)
    # todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
    #  add_mex(${ARGV} ${ARGV}.cpp $<TARGET_OBJECTS:drakeUtil>)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeUtil)
  endmacro()

  add_subdirectory(@PlanarRigidBodyManipulator/private)
  add_planar_rbm_mex(deleteModelpmex)
  add_planar_rbm_mex(doKinematicspmex)
  add_planar_rbm_mex(HandCpmex)
  add_planar_rbm_mex(forwardKinpmex)
  add_planar_rbm_mex(forwardKinVelpmex)

  add_subdirectory(test)

  pods_install_libraries(drakeRBM)
  pods_install_headers(RigidBodyManipulator.h RigidBody.h DESTINATION drake)
  pods_install_pkg_config_file(drake-rbm
    LIBS -ldrakeRBM
    REQUIRES  
    VERSION 0.0.1)

endif(eigen3_FOUND)

