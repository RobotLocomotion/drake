#------------------------------------------------------------------------------
# Add an example. Besides setting up the external project, this also creates
# the option used to control if the example is enabled.
#
# Arguments:
#   <EXAMPLE> - Name of the example.
#   <DEFAULT_ENABLED> - Is the example enabled by default? (`ON` or `OFF`)
#
#   <DOCSTRING>
#       Description of the example, to be used as the documentation string for
#       the example's option.
#------------------------------------------------------------------------------
function(drake_add_example EXAMPLE DEFAULT_ENABLED DOCSTRING)
  string(TOUPPER ${EXAMPLE} EXAMPLE_UPPER)

  # Add option for example
  option(EXAMPLES_${EXAMPLE_UPPER} ${DOCSTRING} ${DEFAULT_ENABLED})

  # Set up example, if enabled
  if(EXAMPLES_${EXAMPLE_UPPER})
    message(STATUS "Installation will include extra example: ${EXAMPLE}")

    # Set up submodule and commands for synchronizing submodule
    drake_add_submodule(drake/examples/${EXAMPLE}
      _ex_DOWNLOAD_COMMAND _ex_UPDATE_COMMAND)

    # Add the external project
    ExternalProject_Add(download-${EXAMPLE}
      SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake/examples/${EXAMPLE}
      DOWNLOAD_COMMAND ${_ex_DOWNLOAD_COMMAND}
      DOWNLOAD_DIR ${PROJECT_SOURCE_DIR}
      UPDATE_COMMAND ${_ex_UPDATE_COMMAND}
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND "")

    # Set up build step to ensure project is updated before build
    drake_forceupdate(download-${EXAMPLE})

    # Add example to download dependencies
    add_dependencies(download-all download-${EXAMPLE})
    add_dependencies(drake download-${EXAMPLE})
  endif()
endfunction()
