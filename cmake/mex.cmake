#------------------------------------------------------------------------------
# Add a MATLAB MEX target to the drake project.
#
#  matlab_add_mex(<name> [EXECUTABLE | MODULE | SHARED] <src1> [<src2> ...]
#------------------------------------------------------------------------------
function(drake_add_mex NAME)
  if(NOT Matlab_FOUND)
    return()
  endif()

  cmake_parse_arguments("" "EXECUTABLE;MODULE;SHARED" "" "" ${ARGN})

  if(_EXECUTABLE)
    set(_type EXECUTABLE)
  elseif(_MODULE)
    set(_type MODULE)
  elseif()
    set(_type SHARED)
  endif()

  matlab_add_mex(NAME ${NAME} ${_type} SRC ${_UNPARSED_ARGUMENTS})
  set_target_properties(${NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(${NAME} ${CMAKE_DL_LIBS})
endfunction()
