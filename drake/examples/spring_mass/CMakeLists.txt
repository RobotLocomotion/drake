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

add_subdirectory(test)
