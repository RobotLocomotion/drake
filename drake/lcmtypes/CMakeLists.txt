include(${LCM_USE_FILE})

if(PYTHONINTERP_FOUND)
  set(python_args PYTHON_SOURCES python_install_sources)
endif()
if(JAVA_FOUND AND TARGET lcm-java)
  set(java_args JAVA_SOURCES java_sources)
endif()

lcm_wrap_types(
  C_SOURCES sources
  C_HEADERS c_install_headers
  CPP_HEADERS cpp_install_headers
  ${python_args}
  ${java_args}
  lcmt_body_motion_data.lcm
  lcmt_body_wrench_data.lcm
  lcmt_drake_signal.lcm
  lcmt_driving_command_t.lcm
  lcmt_euler_floating_joint_state_t.lcm
  lcmt_foot_flag.lcm
  lcmt_force_torque.lcm
  lcmt_idm_with_trajectory_agent_state_t.lcm
  lcmt_iiwa_command.lcm
  lcmt_iiwa_status.lcm
  lcmt_joint_pd_override.lcm
  lcmt_piecewise_polynomial.lcm
  lcmt_polynomial.lcm
  lcmt_polynomial_matrix.lcm
  lcmt_qp_controller_input.lcm
  lcmt_quadrotor_input_t.lcm
  lcmt_quadrotor_output_t.lcm
  lcmt_robot_state.lcm
  lcmt_scope_data.lcm
  lcmt_simple_car_config_t.lcm
  lcmt_simple_car_state_t.lcm
  lcmt_simulation_command.lcm
  lcmt_support_data.lcm
  lcmt_viewer_command.lcm
  lcmt_viewer_draw.lcm
  lcmt_viewer_geometry_data.lcm
  lcmt_viewer_link_data.lcm
  lcmt_viewer_load_robot.lcm
  lcmt_whole_body_data.lcm
  lcmt_zmp_com_observer_state.lcm
  lcmt_zmp_data.lcm
)

add_custom_target(drake_lcmtypes_hpp DEPENDS ${cpp_install_headers})

add_library(drake_lcmtypes STATIC ${sources})
target_link_libraries(drake_lcmtypes lcm)
target_include_directories(drake_lcmtypes INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})

add_library(drakeLCMTypes INTERFACE)
add_dependencies(drakeLCMTypes drake_lcmtypes_hpp)
target_link_libraries(drakeLCMTypes INTERFACE lcm-coretypes)
target_include_directories(drakeLCMTypes INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})

install(FILES ${c_install_headers} DESTINATION include/lcmtypes)
install(FILES ${cpp_install_headers} DESTINATION include/lcmtypes/drake)

install(TARGETS drake_lcmtypes
  ARCHIVE DESTINATION lib${LIB_SUFFIX}
  INCLUDES DESTINATION include
)

if(PYTHONINTERP_FOUND)
  lcm_install_python(${python_install_sources})
endif()

if(JAVA_FOUND)
  if(NOT TARGET lcm-java)
    message(FATAL_ERROR "lcm-java not found; was LCM built with Java support?")
  endif()

  add_jar(lcmtypes_drake-java
    OUTPUT_NAME lcmtypes_drake
    INCLUDE_JARS lcm-java
    SOURCES ${java_sources}
  )

  install_jar(lcmtypes_drake-java share/java)
endif()
