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
  message(STATUS "SPHINX_EXECUTABLE (sphinx-build) not found. Sphinx will be excluded from the documentation build.")
endif()

find_package(Doxygen MODULE)

if(Doxygen_FOUND)
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

  find_package(Perl MODULE)

  if(Perl_FOUND)
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
    message(STATUS "PERL_EXECUTABLE (perl) not found. MATLAB Doxygen will be excluded from the documentation build.")
  endif()
else()
  message(STATUS "DOXYGEN_EXECUTABLE (doxygen) not found. C++ and MATLAB Doxygen will be excluded from the documentation build.")
endif()

set(OXYGEN_DIR "$ENV{OXYGEN_DIR}" CACHE PATH
  "Path to the directory containing the oXygen XML Editor")

if(OXYGEN_DIR AND EXISTS "${OXYGEN_DIR}")
  if(APPLE)
    set(OXYGEN_SCHEMA_DOCUMENTATION_EXECUTABLE
      "${OXYGEN_DIR}/schemaDocumentationMac.sh")
  else()
    set(OXYGEN_SCHEMA_DOCUMENTATION_EXECUTABLE
      "${OXYGEN_DIR}/schemaDocumentation.sh")
  endif()
  configure_file(oxygen_cfg.xml.in oxygen_cfg.xml @ONLY)
  add_custom_command(
    COMMAND ${OXYGEN_SCHEMA_DOCUMENTATION_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/drakeURDF.xsd
            -cfg:${CMAKE_CURRENT_BINARY_DIR}/oxygen_cfg.xml
    COMMENT "Building URDF documentation..."
    OUTPUT urdf_output
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set_property(SOURCE urdf_output PROPERTY SYMBOLIC ON)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/urdf
    DESTINATION ${DRAKE_DOCUMENTATION_DIR} OPTIONAL)
  list(APPEND DRAKE_DOCUMENTATION_OUTPUTS urdf_output)
else()
  message(STATUS "OXYGEN_DIR (oXygen XML Editor) not found. URDF documentation will be excluded from the documentation build.")
endif()

if(DRAKE_DOCUMENTATION_OUTPUTS)
  # BUILD_DOCUMENTATION_ALWAYS is an undocumented option for use by CI builds.
  # When set to ON, documentation is included in the default build targets
  # "all" and "ALL_BUILD".
  # BUILD_DOCUMENTATION_ALWAYS is off by default so that developers do not have
  # to wait for the documentation to build every time they rebuild Drake itself.
  if(BUILD_DOCUMENTATION_ALWAYS)
    set(_DRAKE_DOCUMENTATION_ALL ALL)
  else()
    set(_DRAKE_DOCUMENTATION_ALL "")
  endif()

  add_custom_target(documentation ${_DRAKE_DOCUMENTATION_ALL}
    DEPENDS ${DRAKE_DOCUMENTATION_OUTPUTS})
  install(FILES .nojekyll CNAME DESTINATION ${DRAKE_DOCUMENTATION_DIR})
endif()
