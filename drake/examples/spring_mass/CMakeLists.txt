# Source files used to build drakeSpringMassSystem.
set(sources
    spring_mass_system.cc)

# Headers that are needed by code here but should not
# be exposed anywhere else.
set(private_headers
    spring_mass_system.h)

add_library_with_exports(LIB_NAME drakeSpringMassSystem
  STATIC
  SOURCE_FILES ${sources} ${installed_headers} ${private_headers})
add_dependencies(drakeSpringMassSystem drakeSystemFramework)
target_link_libraries(drakeSpringMassSystem drakeSystemFramework)

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem2 SOURCE_FILES
    lcm_input_system.cc)
  add_dependencies(drakeLCMSystem2 drake_lcmtypes_hpp)
  target_link_libraries(drakeLCMSystem2 lcm)
  if(WIN32)
    target_link_libraries(drakeLCMSystem2 ws2_32)
  endif()

  pods_install_libraries(drakeLCMSystem2)
  drake_install_headers(lcm_input_system.h)
  pods_install_pkg_config_file(drake-lcm-system2
    LIBS -ldrakeLCMSystem2
    REQUIRES lcm
    VERSION 0.0.1)
endif()

add_subdirectory(test)
