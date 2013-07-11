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

function(find_matlab)
  # sets the variables: MATLAB_ROOT, MATLAB_CPU, MEX, MEX_EXT
  #    as well as all of the mexopts

  # first run matlab (if necessary) to find matlabroot and cpu path
  if (NOT EXISTS .matlabroot OR NOT EXISTS .matlabcpu)
    # todo: find_program matlab instead of just calling it?
    execute_process(COMMAND matlab -nodisplay -r "ptr=fopen('.matlabroot','w'); fprintf(ptr,'%s',matlabroot); fclose(ptr); ptr=fopen('.matlabcpu','w'); fprintf(ptr,'%s',lower(computer)); fclose(ptr); exit")
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

  get_mex_option(FC)
  get_mex_option(FFLAGS)
  get_mex_option(FDEBUGFLAGS)
  get_mex_option(FOPTIMFLAGS)
  get_mex_option(FLIBS)
  get_mex_arguments(FC)

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

  message(STATUS MATLAB_ROOT = ${MATLAB_ROOT} )
  message(STATUS MEX_CC_ARGUMENTS = ${MEX_CC_ARGUMENTS} )
  include_directories( ${MATLAB_ROOT}/extern/include ${MATLAB_ROOT}/simulink/include )
#  set(full_target ${target}.${MEX_EXT})

  add_library(${target} MODULE ${ARGV}) 
#  add_custom_command(OUTPUT ${target} COMMAND ${MEX} -o ${full_target} ${ARGV} WORKING_DIRECTORY .)
  set_target_properties(${target} PROPERTIES PREFIX "" SUFFIX  ".${MEX_EXT}" )

  # set up compile/link flags 
  # TODO: actually mine these from mexopts.sh (but have to get the search path correct)
  set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-DMATLAB_MEX_FILE -fno-common -fexceptions -DMX_COMPAT_32")
  set_target_properties(${target} PROPERTIES COMPILE_FLAGS_RELEASE "-DNDEBUG")
  set_target_properties(${target} PROPERTIES LINK_FLAGS "-exported_symbols_list,${MATLAB_ROOT}/extern/lib/${MATLAB_CPU}/mexFunction.map -L${MATLAB_ROOT}/bin/${MATLAB_CPU} -lmx -lmex -lmat -lstdc++")

endfunction()