set(DRAKE_DOCUMENTATION_OUTPUTS "")
set(DRAKE_DOCUMENTATION_DIR ${CMAKE_INSTALL_PREFIX}/share/doc)

find_program(SPHINX_EXECUTABLE
  NAMES sphinx-build
  DOC "Sphinx documentation generator (sphinx-build)")

if(SPHINX_EXECUTABLE)
  configure_file(conf.py.in conf.py @ONLY)
  add_custom_command(
    COMMAND ${SPHINX_EXECUTABLE}
            -b html
            -c ${CMAKE_CURRENT_BINARY_DIR}
            -d ${CMAKE_CURRENT_BINARY_DIR}/doctrees
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_BINARY_DIR}/sphinx
    COMMENT "Building Sphinx documentation..."
    OUTPUT sphinx_output
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set_property(SOURCE sphinx_output PROPERTY SYMBOLIC ON)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/sphinx/
    DESTINATION ${DRAKE_DOCUMENTATION_DIR} OPTIONAL)
  list(APPEND DRAKE_DOCUMENTATION_OUTPUTS sphinx_output)
else()
  message(WARNING "SPHINX_EXECUTABLE (sphinx-build) not found. Sphinx documentation will not be built.")
endif()

find_package(Doxygen)

if(DOXYGEN_FOUND)
  configure_file(Doxyfile_CXX.in Doxyfile_CXX @ONLY)
  add_custom_command(
    COMMAND ${DOXYGEN_EXECUTABLE}
            ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_CXX
    COMMENT "Building C++ Doxygen documentation..."
    OUTPUT doxygen_cxx_output
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set_property(SOURCE doxygen_cxx_output PROPERTY SYMBOLIC ON)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doxygen_cxx/html/
    DESTINATION ${DRAKE_DOCUMENTATION_DIR}/doxygen_cxx OPTIONAL)
  list(APPEND DRAKE_DOCUMENTATION_OUTPUTS doxygen_cxx_output)

  find_package(Perl)

  if(PERL_FOUND)
    configure_file(Doxyfile_MATLAB.in Doxyfile_MATLAB @ONLY)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/DoxygenMatlab
      DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    add_custom_command(
      COMMAND ${DOXYGEN_EXECUTABLE}
              ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_MATLAB
      COMMENT "Building MATLAB Doxygen documentation..."
      OUTPUT doxygen_matlab_output
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    set_property(SOURCE doxygen_matlab_output PROPERTY SYMBOLIC ON)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doxygen_matlab/html/
      DESTINATION ${DRAKE_DOCUMENTATION_DIR}/doxygen_matlab OPTIONAL)
    list(APPEND DRAKE_DOCUMENTATION_OUTPUTS doxygen_matlab_output)
  else()
    message(WARNING "PERL_EXECUTABLE (perl) not found. MATLAB Doxygen documentation will not be built.")
  endif()
else()
  message(WARNING "DOXYGEN_EXECUTABLE (doxygen) not found. C++ and MATLAB Doxygen documentation will not be built.")
endif()

if(ENV{OXYGEN_DIR})
  if(APPLE)
    set(OXYGEN_SCHEMA_DOCUMENTATION_EXECUTABLE
      $ENV{OXYGEN_DIR}/schemaDocumentationMac.sh)
  else()
    set(OXYGEN_SCHEMA_DOCUMENTATION_EXECUTABLE
      $ENV{OXYGEN_DIR}/schemaDocumentation.sh)
  endif()
  configure_file(oxygen_cfg.xml.in oxygen_cfg.xml @ONLY)
  add_custom_command(
    COMMAND ${OXYGEN_SCHEMA_DOCUMENTATION_EXECUTABLE}
            ${CMAKE_CURRENT_BINARY_DIR}/drakeURDF.xsd
            -cfg:${CMAKE_CURRENT_BINARY_DIR}/oxygen_cfg.xml
    COMMENT "Building URDF documentation..."
    OUTPUT urdf_output
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set_property(SOURCE urdf_output PROPERTY SYMBOLIC ON)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/urdf
    DESTINATION ${DRAKE_DOCUMENTATION_DIR} OPTIONAL)
  list(APPEND DRAKE_DOCUMENTATION_OUTPUTS urdf_output)
endif()

if(DRAKE_DOCUMENTATION_OUTPUTS)
  # Undocumented option for use by CI builds to build documentation
  # as part of the default build targets ("all" or "ALL_BUILD").
  # We do not do this by default because developers should only
  # get documentation built by an explicit request for the target.
  if(BUILD_DOCUMENTATION_ALWAYS)
    set(_DRAKE_DOCUMENTATION_ALL ALL)
  else()
    set(_DRAKE_DOCUMENTATION_ALL "")
  endif()

  add_custom_target(documentation ${_DRAKE_DOCUMENTATION_ALL}
    DEPENDS ${DRAKE_DOCUMENTATION_OUTPUTS})
  install(FILES .nojekyll CNAME DESTINATION ${DRAKE_DOCUMENTATION_DIR})
endif()
