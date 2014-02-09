# Macros to handle compilation of MATLAB mex files.
#

#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

macro(get_mex_option option)
  # writes MEX_${option} 

  # todo: do the string parsing using CMAKE commands to make it less platform dependent
  execute_process(COMMAND ${MATLAB_ROOT}/bin/mex -v COMMAND grep ${option} COMMAND head -n 1 COMMAND cut -d "=" -f2 OUTPUT_VARIABLE value ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(STRIP ${value} svalue)
  set(MEX_${option} ${svalue} PARENT_SCOPE)
endmacro()

macro(get_mex_arguments afterstring)
  # writes MEX_${afterstring}_ARGUMENTS 

  # todo: do the string parsing using CMAKE commands to make it less platform dependent
  execute_process(COMMAND ${MATLAB_ROOT}/bin/mex -v COMMAND sed -e "1,/${afterstring}/d" COMMAND grep arguments COMMAND head -n 1 COMMAND cut -d "=" -f2 OUTPUT_VARIABLE value ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(MEX_${afterstring}_ARGUMENTS ${value} PARENT_SCOPE)
endmacro()

function(mex_setup)
  # sets the variables: MATLAB_ROOT, MATLAB_CPU, MEX, MEX_EXT
  #    as well as all of the mexopts

  find_program(matlab matlab)
  if ( NOT matlab )
     message(FATAL_ERROR "Could not find matlab executable")
  endif()
  execute_process(COMMAND ${matlab} -n COMMAND grep -e "MATLAB \\+=" COMMAND cut -d "=" -f2 OUTPUT_VARIABLE _matlab_root)
  if (NOT _matlab_root)
    message(FATAL_ERROR "Failed to extract MATLAB_ROOT from running matlab -n")
  endif()
  string(STRIP ${_matlab_root} MATLAB_ROOT)

  execute_process(COMMAND ${matlab} -n COMMAND grep "ARCH" COMMAND cut -d "=" -f2 OUTPUT_VARIABLE _matlab_cpu)
  if (NOT _matlab_cpu)
    message(FATAL_ERROR "Failed to extract MATLAB_CPU from running matlab -n")
  endif()
  string(STRIP ${_matlab_cpu} MATLAB_CPU)

  execute_process(COMMAND ${MATLAB_ROOT}/bin/mexext OUTPUT_VARIABLE MEX_EXT OUTPUT_STRIP_TRAILING_WHITESPACE)

  if (NOT MATLAB_ROOT OR NOT MATLAB_CPU OR NOT MEX_EXT)
     message(FATAL_ERROR "could not find MATLAB root")
  endif()

#  set(MATLAB_BIN ${MATLAB_ROOT}/bin/${MATLAB_CPU})

  set(MATLAB_ROOT ${MATLAB_ROOT} PARENT_SCOPE)
  set(MATLAB_CPU ${MATLAB_CPU} PARENT_SCOPE)
  set(MEX ${MATLAB_ROOT}/bin/mex PARENT_SCOPE)
  set(MEX_EXT ${MEX_EXT} PARENT_SCOPE)

  get_mex_option(CC)

  get_mex_option(CFLAGS)
  get_mex_option(CDEBUGFLAGS)
  get_mex_option(COPTIMFLAGS)
  get_mex_option(CLIBS)
  get_mex_arguments(CC)
  
  get_mex_option(CXX)
  get_mex_option(CXXDEBUGFLAGS)
  get_mex_option(CXXOPTIMFLAGS)
  get_mex_option(CXXLIBS)
  get_mex_arguments(CXX)

#  not supporting fortran yet below, so might as well comment these out
#  get_mex_option(FC)
#  get_mex_option(FFLAGS)
#  get_mex_option(FDEBUGFLAGS)
#  get_mex_option(FOPTIMFLAGS)
#  get_mex_option(FLIBS)
#  get_mex_arguments(FC)

  get_mex_option(LD)
  get_mex_option(LDFLAGS)
  get_mex_option(LDDEBUGFLAGS)
  get_mex_option(LDOPTIMFLAGS)
  get_mex_option(LDEXTENSION)
  get_mex_arguments(LD)

#  note: skipping LDCXX (and just always use LD)

endfunction()

function(add_mex)
  # useage:  add_mex(target source1 source2 [SHARED,EXECUTABLE]) 
  # note: builds the mex file inplace (not into some build directory)
  # if SHARED is passed in, then it doesn't expect a mexFunction symbol to be defined, and compiles it to e.g., libtarget.so, for eventual linking against another mex file
  # if EXECUTABLE is passed in, then it adds an executable target, which is linked against the appropriate matlab libraries.

  list(GET ARGV 0 target)
  list(REMOVE_AT ARGV 0)

  if (NOT MATLAB_ROOT OR NOT MATLAB_CPU OR NOT MEX_EXT)
     message(FATAL_ERROR "MATLAB not found (or MATLAB_ROOT/MATLAB_CPU not properly parsed)")
  endif()

  include_directories( ${MATLAB_ROOT}/extern/include ${MATLAB_ROOT}/simulink/include )

  # todo error if CMAKE_BUILD_TYPE is not Debug or Release

  # set up compiler/linker options
  # NOTE: due to a CMAKE quirk, the compiler flags for each build type
  # need to be set via the global variables, while the linker flags
  # need to be set using set_target_properties

  # backup global props
  set (CMAKE_C_FLAGS_DEBUG_BK ${CMAKE_C_FLAGS_DEBUG})
  set (CMAKE_C_FLAGS_RELEASE_BK ${CMAKE_C_FLAGS_RELEASE})
  set (CMAKE_CXX_COMPILER_BK ${CMAKE_CXX_COMPILER})
  set (CMAKE_CXX_FLAGS_DEBUG_BK ${CMAKE_CXX_FLAGS_DEBUG})
  set (CMAKE_CXX_FLAGS_RELEASE_BK ${CMAKE_CXX_FLAGS_RELEASE})  

  # set global props
  set (CMAKE_C_FLAGS_DEBUG ${MEX_CFLAGS} ${MEX_CDEBUGFLAGS} ${MEX_CC_ARGUMENTS})
  set (CMAKE_C_FLAGS_RELEASE ${MEX_CFLAGS} ${MEX_COPTIMFLAGS} ${MEX_CC_ARGUMENTS})
  find_program (CMAKE_CXX_COMPILER ${MEX_CXX})
  set (CMAKE_CXX_FLAGS_DEBUG ${MEX_CXXFLAGS} ${MEX_CXXDEBUGFLAGS} ${MEX_CXX_ARGUMENTS})
  set (CMAKE_CXX_FLAGS_RELEASE ${MEX_CXXFLAGS} ${MEX_CXXOPTIMFLAGS} ${MEX_CXX_ARGUMENTS})

  list(FIND ARGV SHARED isshared)
  list(FIND ARGV EXECUTABLE isexe)
  if (isshared EQUAL -1)
    if (isexe EQUAL -1)
        add_library(${target} MODULE ${ARGV})
    else()
	list(REMOVE_ITEM ARGV EXECUTABLE)
	add_executable(${target} ${ARGV})
    endif()
  else()
    add_library(${target} ${ARGV})
  endif()

  # restore global props
  set (CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG_BK})  
  set (CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE_BK})  
  set (CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER_BK})
  set (CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG_BK})  
  set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE_BK})  

  set_target_properties(${target} PROPERTIES 
    COMPILE_FLAGS "-DMATLAB_MEX_FILE" 
    )

  if (isexe GREATER -1)
    # see note below
    if (NOT TARGET exelast) 
      set(dummy_c_file ${CMAKE_CURRENT_BINARY_DIR}/dummy.c)
      add_custom_command(COMMAND ${CMAKE_COMMAND} -E touch ${dummy_c_file}
                         OUTPUT ${dummy_c_file})
      add_library(exelast STATIC ${dummy_c_file})
      target_link_libraries(exelast "${MEX_CLIBS} -ldl")  # note: the -ldl here might be overkill?  so far only needed it for drake_debug_mex.  (but it has to come later in the compiler arguments, too, in order to work.
    endif()

    target_link_libraries(${target} exelast)
  elseif (isshared GREATER -1)
    set_target_properties(${target} PROPERTIES
      LINK_FLAGS "${MEX_CLIBS}"
      )
  else()
    # note: on ubuntu, gcc did not like the MEX_CLIBS coming along with LINK_FLAGS (it only works if they appear after the input files).  this is a nasty trick that I found online
    if (NOT TARGET last) 
      set(dummy_c_file ${CMAKE_CURRENT_BINARY_DIR}/dummy.c)
      add_custom_command(COMMAND ${CMAKE_COMMAND} -E touch ${dummy_c_file}
                         OUTPUT ${dummy_c_file})
      add_library(last STATIC ${dummy_c_file})
      target_link_libraries(last ${MEX_CLIBS})
    endif()

    set_target_properties(${target} PROPERTIES 
      PREFIX ""
      SUFFIX ".${MEX_EXT}"
      LINK_FLAGS "${MEX_LDFLAGS} ${MEX_LD_ARGUMENTS}" # -Wl,-rpath ${CMAKE_INSTALL_PREFIX}/lib"  
      LINK_FLAGS_DEBUG	"${MEX_LDDEBUGFLAGS}"
      LINK_FLAGS_RELEASE	"${MEX_LDOPTIMFLAGS}"
      ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"   
      )
    target_link_libraries(${target} last)
  endif()

  # todo: add CLIBS or CXXLIBS to LINK_FLAGS selectively based in if it's a c or cxx target (always added CXX above)

endfunction()

## calls compilers with --version option and checks the output 
# calls compilers with -dumpversion and checks the output
# (because it appears that ccache messes with the --version output)
# returns TRUE if the strings match or FALSE if they don't.  
#   (note: you can use  if (outvar) to test )
# this seems to be a more robust and less complex method than trying to call xcrun -find, readlink to follow symlinks, etc.
function(compare_compilers outvar compiler1 compiler2)

  separate_arguments(c1_args UNIX_COMMAND ${compiler1}) 
  list(APPEND c1_args "-dumpversion")
  execute_process(COMMAND ${c1_args} OUTPUT_VARIABLE c1_ver OUTPUT_STRIP_TRAILING_WHITESPACE)

  separate_arguments(c2_args UNIX_COMMAND ${compiler2})
  list(APPEND c2_args "-dumpversion")
  execute_process(COMMAND ${c2_args} OUTPUT_VARIABLE c2_ver OUTPUT_STRIP_TRAILING_WHITESPACE)

  if (c1_ver AND c2_ver AND "${c1_ver}" STREQUAL "${c2_ver}")
    set(${outvar} TRUE PARENT_SCOPE)
  else()
    set(${outvar} FALSE PARENT_SCOPE)
    message(STATUS "compiler1 version string:\n${c1_ver}")
    message(STATUS "compiler2 version string:\n${c2_ver}")
  endif()

endfunction()


mex_setup()

compare_compilers(compilers_match "${MEX_CC}" "${CMAKE_C_COMPILER}")
if (NOT compilers_match)
   message(FATAL_ERROR "Your cmake C compiler is: ${CMAKE_C_COMPILER} but your mex options use: ${MEX_CC} .  Consider rerunning 'mex -setup' in  Matlab.")
endif()

compare_compilers(compilers_match ${MEX_CXX} ${CMAKE_CXX_COMPILER})
if (NOT compilers_match)
   message(FATAL_ERROR "Your cmake CXX compiler is: ${CMAKE_CXX_COMPILER} but your mex options end up pointing to: ${MEX_CXX} .  Consider rerunning 'mex -setup' in Matlab.")
endif()

# NOTE:  would like to check LD also, but it appears to be difficult with cmake  (there is not explicit linker executable variable, only the make rule), and  even my mex code assumes that LD==LDCXX for simplicity.