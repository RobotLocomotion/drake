# Macros to handle compilation of MATLAB mex files.
#

#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

macro(get_mex_option option)
  # writes MEX_${option} 
  execute_process(COMMAND ${MATLAB_ROOT}/bin/mex -v COMMAND grep ${option} COMMAND head -n 1 COMMAND cut -d "=" -f2 OUTPUT_VARIABLE value ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(MEX_${option} ${value} PARENT_SCOPE)
endmacro()

macro(get_mex_arguments afterstring)
  # writes MEX_${afterstring}_ARGUMENTS 
  execute_process(COMMAND ${MATLAB_ROOT}/bin/mex -v COMMAND sed -e "1,/${afterstring}/d" COMMAND grep arguments COMMAND head -n 1 COMMAND cut -d "=" -f2 OUTPUT_VARIABLE value ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(MEX_${afterstring}_ARGUMENTS ${value} PARENT_SCOPE)
endmacro()

function(mex_setup)
  # sets the variables: MATLAB_ROOT, MATLAB_CPU, MEX, MEX_EXT
  #    as well as all of the mexopts

  # first run matlab (if necessary) to find matlabroot and cpu path
  if (NOT EXISTS .matlabroot OR NOT EXISTS .matlabcpu)
    find_program(matlab matlab)
    execute_process(COMMAND ${matlab} -nodisplay -r "ptr=fopen('.matlabroot','w'); fprintf(ptr,'%s',matlabroot); fclose(ptr); ptr=fopen('.matlabcpu','w'); fprintf(ptr,'%s',lower(computer)); fclose(ptr); exit")
  endif()  

  execute_process(COMMAND cat .matlabroot OUTPUT_VARIABLE MATLAB_ROOT)
  execute_process(COMMAND cat .matlabcpu OUTPUT_VARIABLE MATLAB_CPU)
  execute_process(COMMAND ${MATLAB_ROOT}/bin/mexext OUTPUT_VARIABLE MEX_EXT OUTPUT_STRIP_TRAILING_WHITESPACE)

  set(MATLAB_BIN ${MATLAB_ROOT}/bin/${MATLAB_CPU})

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

#  set(MATLAB_EXT_LIB ${MATLAB_ROOT}/sys/os/${MATLAB_CPU})
#  find_library(MATLAB_MEX_LIBRARY mex ${MATLAB_BIN} )
#  find_library(MATLAB_MX_LIBRARY  mx  ${MATLAB_BIN} )
#  find_library(MATLAB_ENG_LIBRARY eng ${MATLAB_BIN} )
	 
endfunction()

function(add_mex)
  # useage:  add_mex(target source1 source2) # ... DEPENDS depend1 depend2 ) 
  # note: builds the mex file inplace (not into some build directory)
  list(GET ARGV 0 target)
  list(REMOVE_AT ARGV 0)

  # todo: error if mex_setup hasn't been called

#  message(STATUS MATLAB_ROOT = ${MATLAB_ROOT} )
  include_directories( ${MATLAB_ROOT}/extern/include ${MATLAB_ROOT}/simulink/include )

  # todo error if CMAKE_BUILD_TYPE is not Debug or Release

  # set up compiler/linker options
  # NOTE: due to a CMAKE quirk, the compiler flags for each build type
  # need to be set via the global variables, while the linker flags
  # need to be set using set_target_properties

  # backup global props
  set (CMAKE_C_FLAGS_DEBUG_BK ${CMAKE_C_FLAGS_DEBUG})
  set (CMAKE_C_FLAGS_RELEASE_BK ${CMAKE_C_FLAGS_RELEASE})
  set (CMAKE_CXX_FLAGS_DEBUG_BK ${CMAKE_CXX_FLAGS_DEBUG})
  set (CMAKE_CXX_FLAGS_RELEASE_BK ${CMAKE_CXX_FLAGS_RELEASE})  

  # set global props
  set (CMAKE_C_FLAGS_DEBUG ${MEX_CFLAGS} ${MEX_CDEBUGFLAGS} ${MEX_CC_ARGUMENTS})
  set (CMAKE_C_FLAGS_RELEASE ${MEX_CFLAGS} ${MEX_COPTIMFLAGS} ${MEX_CC_ARGUMENTS})
  set (CMAKE_CXX_FLAGS_DEBUG ${MEX_CXXFLAGS} ${MEX_CXXDEBUGFLAGS} ${MEX_CXX_ARGUMENTS})
  set (CMAKE_CXX_FLAGS_RELEASE ${MEX_CXXFLAGS} ${MEX_CXXOPTIMFLAGS} ${MEX_CXX_ARGUMENTS})

  add_library(${target} MODULE ${ARGV}) 

  # restore global props
  set (CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG_BK})  
  set (CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE_BK})  
  set (CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG_BK})  
  set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE_BK})  

  set_target_properties(${target} PROPERTIES 
    PREFIX ""
    SUFFIX ".${MEX_EXT}"
    COMPILE_FLAGS "-DMATLAB_MEX_FILE" 
    LINK_FLAGS "${MEX_LDFLAGS} ${MEX_LD_ARGUMENTS} ${MEX_CXXLIBS}"  
    LINK_FLAGS_DEBUG	"${MEX_LDDEBUGFLAGS}"
    LINK_FLAGS_RELEASE	"${MEX_LDOPTIMFLAGS}"
    )

  # todo: add CLIBS or CXXLIBS to LINK_FLAGS selectively based in if it's a c or cxx target (always added CXX above)

endfunction()