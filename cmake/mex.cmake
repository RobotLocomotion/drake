# Macros to handle compilation of MATLAB mex files.
#

cmake_minimum_required(VERSION 2.8.3)
# for cmake_parse_arguments

macro(get_mex_option option_name)
  # usage: get_mex_option(option_name [NAMES names_to_try_in_order REQUIRED])
  # writes MEX_${option_name}
  set(ARG_NAMES "")
  set(ARG_REQUIRED "")
  if ( ${ARGC} GREATER 1 )
    cmake_parse_arguments(ARG "REQUIRED" "" "NAMES" ${ARGN})
  endif()
  if ( NOT ARG_NAMES )
    set(ARG_NAMES ${option_name})
  endif()

  set(svalue "")
  foreach (name ${ARG_NAMES})
    string(REGEX MATCH "${name}[^\r\n]*" option_line ${mexv_output}) # first line containing ${name}
    if ( option_line )
       string(REGEX REPLACE "[^=:]+[=:](.*)" "\\1" value ${option_line})  # replace entire string with capturing group (after = )
       string(STRIP ${value} svalue)
       break()
    endif()
  endforeach()

  if ( svalue )
    string(REGEX REPLACE "^\"(.*)\"$" "\\1" svalue ${svalue})  # starting in matlab 2014b, at least on mac, i needed to remove quotes from around the result
    set(MEX_${option_name} ${svalue})
    set(MEX_${option_name} ${svalue} PARENT_SCOPE)
#    message(STATUS "MEX_${option_name} = ${svalue}")
  else()
    if ( ARG_REQUIRED )
      message(FATAL_ERROR "Could not find MEX_${option_name} using mex -v")
    else()
     set(MEX_${option_name} "")
     set(MEX_${option_name} "" PARENT_SCOPE)
    endif()
  endif()
endmacro()

macro(get_mex_arguments afterstring)
  # writes MEX_${afterstring}_ARGUMENTS

  cmake_parse_arguments(ARG "REQUIRED" "" "" ${ARGN})
  set(arguments_name MEX_${afterstring}_ARGUMENTS)

  string(REGEX MATCH "${afterstring}.*" starting_with_afterstring ${mexv_output}) # everything starting with afterstring
  if ( starting_with_afterstring )
    string(REGEX MATCH "arguments[^\r\n]*" arguments_line ${starting_with_afterstring}) # first line containing arguments
    if ( arguments_line )
      string(REGEX REPLACE "[^=]+=(.*)" "\\1" value ${arguments_line}) # replace entire string with capturing group (after =)
      string(STRIP ${value} svalue)
      set(${arguments_name} ${svalue} PARENT_SCOPE)
#      message(STATUS "${arguments_name} = ${svalue}")
    else()
      if ( ARG_REQUIRED )
        message(FATAL_ERROR "Could not find arguments line for ${afterstring} using mex -v")
      endif()
      set(${arguments_name} "" PARENT_SCOPE)
    endif()
  else()
    if (ARG_REQUIRED)
      message(WARNING "Could not find block containing arguments for ${afterstring} using mex -v")
    endif()
    set(${arguments_name} "" PARENT_SCOPE)
  endif()
endmacro()

function(mex_setup)
  # usage: mex_setup([REQUIRED])
  # sets the variables: MATLAB_ROOT, MEX, MEX_EXT
  #    as well as all of the mexopts

  list(FIND ARGV REQUIRED isrequired)

  find_program(matlab matlab)
  if ( NOT matlab )
    if (isrequired GREATER -1)
      message(FATAL_ERROR "Could NOT find MATLAB main program executable")
    endif()
    message(STATUS "Could NOT find MATLAB main program executable. MEX support will be disabled.")
    return()
  endif()

  set(matlab "${matlab}" CACHE FILEPATH "MATLAB main program executable")

  # logic from upstream FindMatlab.cmake...

  # resolve symlinks
  get_filename_component(MATLAB_ROOT "${matlab}" REALPATH)

  # get the directory (the command below has to be run twice)
  # this will be the matlab root
  get_filename_component(MATLAB_ROOT "${MATLAB_ROOT}" DIRECTORY)
  get_filename_component(MATLAB_ROOT "${MATLAB_ROOT}" DIRECTORY) # Matlab should be in bin

  find_program(mex NAMES mex mex.bat HINTS ${MATLAB_ROOT}/bin NO_DEFAULT_PATH)
  if (NOT mex)
     message(FATAL_ERROR "Could NOT find MEX compiler")
  endif()

  find_program(mexext NAMES mexext mexext.bat HINTS ${MATLAB_ROOT}/bin)
  if (mexext)
    execute_process(COMMAND ${mexext} OUTPUT_VARIABLE MEX_EXT OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif()
  if (NOT MEX_EXT)
     message(FATAL_ERROR "Could NOT determine binary MEX filename extension")
  endif()

  find_file(simulink_FOUND NAMES simstruc.h HINTS ${MATLAB_ROOT}/simulink/include)

  set(MATLAB_ROOT ${MATLAB_ROOT} PARENT_SCOPE)
  set(mex ${mex} PARENT_SCOPE)
  set(MEX_EXT ${MEX_EXT} PARENT_SCOPE)
  set(simulink_FOUND ${simulink_FOUND} PARENT_SCOPE)

  # Try to compile a file using MEX. The verbose output from mex -v includes the
  # values of the compiler and linker flags that it uses to so.
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/dummy_mex.c "")
  execute_process(COMMAND ${mex} -largeArrayDims -v ${CMAKE_CURRENT_BINARY_DIR}/dummy_mex.c
    OUTPUT_VARIABLE mexv_output ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

  # Parse the output from the try-compile (mexv_output) to obtain the compiler
  # and linker flags. This creates variables MEX_* in parent scope that are used
  # to set these flags in the add_mex() function.
  if ( WIN32 )
    get_mex_option(CC NAMES COMPILER REQUIRED)
    get_mex_option(CXX NAMES COMPILER REQUIRED)  # cl

    get_mex_option(CFLAGS NAMES COMPFLAGS)
    get_mex_option(CXXFLAGS NAMES COMPFLAGS)  # /MD /nologo ...
    get_mex_option(DEFINES NAMES COMPDEFINES)  # /D_CRT_SECURE_NO_DEPRECATE ...
    get_mex_option(MATLABMEX)  # /DMATLAB_MEX_FILE ...
    get_mex_option(COPTIMFLAGS NAMES OPTIMFLAGS)
    get_mex_option(CXXOPTIMFLAGS NAMES OPTIMFLAGS)  # /O2 /DNDEBUG /Zp8 ...
    get_mex_option(CDEBUGFLAGS NAMES DEBUGFLAGS)
    get_mex_option(CXXDEBUGFLAGS NAMES DEBUGFLAGS)  # /Z7 ...

    get_mex_option(LD NAMES LINKER)  # link
    get_mex_option(LDFLAGS NAMES LINKFLAGS)  # /nologo ...
    get_mex_option(LINKLIBS)  # /LIBPATH:$MATLAB_ROOT\extern\lib\$ARCH\microsoft libmx.lib libmex.lib libmat.lib ...
    get_mex_option(LINKEXPORT)
    get_mex_option(LDDEBUGFLAGS NAMES LINKDEBUGFLAGS)  # /debug ...

    if (MSVC)
#        string(REGEX REPLACE "^.*implib:\"(.*)templib.x\" .*$" "\\1" tempdir "${MEX_LDFLAGS}")
#    	if (tempdir)
#       	  execute_process(COMMAND mkdir ${tempdir})
#          message("Creating temporary directory: ${tempdir}")
#    	endif()

        # newer version of matlab set the alignment, which (I believe) is interfering with the eigen alignment
        # zap that compiler argument
        string(REGEX REPLACE "/Zp[0-9]*" "" MEX_CFLAGS "${MEX_CFLAGS}")
        string(REGEX REPLACE "/Zp[0-9]*" "" MEX_CXXFLAGS "${MEX_CXXFLAGS}")

        string(REGEX REPLACE "/implib:.*templib.x\"" "" MEX_LDFLAGS "${MEX_LDFLAGS}")
	string(REGEX REPLACE "/MAP:\"[.a-zA-Z0-9]*\"" "" MEX_LDFLAGS "${MEX_LDFLAGS}")
	string(REGEX REPLACE "/PDB:\"[.a-xA-Z0-9]*\"" "" MEX_LDDEBUGFLAGS "{MED_LDDEBUGFLAGS}")
	set(MEX_LDFLAGS "${MEX_LDFLAGS}" PARENT_SCOPE)
#        message(STATUS MEX_LDFLAGS=${MEX_LDFLAGS})
    endif()
  else()
    get_mex_option(CC REQUIRED)

#    get_mex_option(CFLAGS REQUIRED)
    get_mex_option(CXXFLAGS NAMES CXXFLAGS CFLAGS REQUIRED)  # -fexceptions -fPIC ...
    get_mex_option(DEFINES)  # -D_GNU_SOURCE ...
    get_mex_option(MATLABMEX)  # -DMATLAB_MEX_FILE ...
    get_mex_option(INCLUDE)  # -I$MATLAB_ROOT/extern/include -I$MATLAB_ROOT/simulink/include ...
    get_mex_option(CDEBUGFLAGS)
    get_mex_option(COPTIMFLAGS)
    get_mex_option(CLIBS)
    get_mex_arguments(CC)

    get_mex_option(CXX NAMES CXX CC REQUIRED)  # g++, xcrun -sdk $SDK_VERSION clang++ ...
    get_mex_option(CXXDEBUGFLAGS)  # -g ...
    get_mex_option(CXXOPTIMFLAGS)  # -O2 -DNDEBUG ...
    get_mex_option(CXXLIBS)
    get_mex_arguments(CXX)

#  not supporting fortran yet below, so might as well comment these out
#  get_mex_option(FC)
#  get_mex_option(FFLAGS)
#  get_mex_option(FDEBUGFLAGS)
#  get_mex_option(FOPTIMFLAGS)
#  get_mex_option(FLIBS)
#  get_mex_arguments(FC)

    get_mex_option(LD REQUIRED)  # ld
    get_mex_option(LDFLAGS REQUIRED)
    get_mex_option(LINKLIBS)  # -L$MATLAB_ROOT/bin/$ARCH -lmx -lmex -lmat ...
    get_mex_option(LDDEBUGFLAGS)  # -g ...
    get_mex_option(LDOPTIMFLAGS)  # -O ...
    get_mex_option(LDEXTENSION NAMES LDEXTENSION LDEXT REQUIRED)
    get_mex_arguments(LD)

#  note: skipping LDCXX (and just always use LD)
  endif()

  # figure out LDFLAGS for exes and shared libraries
  set (MEXLIB_LDFLAGS ${MEX_LDFLAGS} ${MEX_LD_ARGUMENTS} ${MEX_CXXLIBS} ${MEX_LINKLIBS} ${MEX_LINKEXPORT})

  if (NOT WIN32) # AND (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX))
    set(MEXLIB_LDFLAGS ${MEXLIB_LDFLAGS} "-ldl")
  endif()

  string(REPLACE "-bundle" "" MEXLIB_LDFLAGS "${MEXLIB_LDFLAGS}")
  string(REGEX REPLACE "[ ;][^ ;]*mexFunction.map\"*" "" MEXLIB_LDFLAGS "${MEXLIB_LDFLAGS}")  # zap the exports definition file
  string(REPLACE ";" " " MEXLIB_LDFLAGS "${MEXLIB_LDFLAGS}")

#  string(REGEX MATCH "-L[^ ]*" MEX_RPATH ${MEXLIB_LDFLAGS})
#  set(MEX_RPATH ${MEX_RPATH} PARENT_SCOPE)

#  # manually find full library path
#  string(REGEX MATCH "-L[^ ]*" __ld_dir ${MEXLIB_LDFLAGS})  # I'm expecting only one lib path
#  string(REGEX REPLACE "^-L" "" __ld_dir ${__ld_dir})
#  string(REPLACE "\"" "" __ld_dir ${__ld_dir})  # remove quotes (will this work on windows?)
#  string(REGEX MATCHALL "-l[^ ]*" __libstr ${MEXLIB_LDFLAGS})
#  foreach(__lib ${__libstr})
#    string(REPLACE "-l" "" __lib "${__lib}")
#    find_library("my${__lib}" NAMES "${__lib}" PATHS ${__ld_dir} NO_DEFAULT_PATH)
#    find_library(my${__lib} NAMES "${__lib}")
#    message(STATUS "replacing -l${__lib} with ${my${__lib}}")
#    string(REPLACE "-l${__lib}" "${my${__lib}}" MEXLIB_LDFLAGS ${MEXLIB_LDFLAGS})
#  endforeach()


  # todo: handle C separately from CXX?
  set (MEX_COMPILE_FLAGS "${MEX_CXXFLAGS} ${MEX_DEFINES} ${MEX_CXX_ARGUMENTS}")

  # CMake will add the appropriate flags for the chosen build configuration
  # instead of these flags that were parsed from the output of the try-compile.
  if(MSVC)
    # Controlled by CMAKE_BUILD_TYPE. MEX always uses "/MD", but that would
    # conflict with "/MDd" which is set by CMake during "Debug" builds.
    string(REPLACE "/MD " "" MEX_COMPILE_FLAGS "${MEX_COMPILE_FLAGS}")
    # Controlled by CMAKE_VERBOSE_MAKEFILE even for generators such as
    # "Visual Studio 2015".
    string(REPLACE "/nologo " "" MEX_COMPILE_FLAGS "${MEX_COMPILE_FLAGS}")
    string(REPLACE "/nologo " "" MEXLIB_LDFLAGS "${MEXLIB_LDFLAGS}")
  else()
    # Controlled by CMAKE_C_STANDARD and CMAKE_CXX_STANDARD. MEX always sets the
    # C dialect to ANSI and the C++ dialect to either ANSI or C++11 depending on
    # the version of MATLAB, but these conflict if CMAKE_C_STANDARD or
    # CMAKE_CXX_STANDARD are set in order to enable C++14 or above.
    string(REPLACE "-ansi " "" MEX_COMPILE_FLAGS "${MEX_COMPILE_FLAGS}")
    string(REPLACE "-std=c++11 " "" MEX_COMPILE_FLAGS "${MEX_COMPILE_FLAGS}")
  endif()

  set(MEX_COMPILE_FLAGS "${MEX_COMPILE_FLAGS}" PARENT_SCOPE)

  # note: on ubuntu, gcc did not like the MEX_CLIBS coming along with LINK_FLAGS (it only works if they appear after the  input files).  this is a nasty trick that I found online
  if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(dummy_c_file ${CMAKE_CURRENT_BINARY_DIR}/dummy.c)
    add_custom_command(COMMAND ${CMAKE_COMMAND} -E touch ${dummy_c_file}
  			OUTPUT ${dummy_c_file})


    add_library(liblast STATIC ${dummy_c_file})
    target_link_libraries(liblast ${MEXLIB_LDFLAGS})
  endif()

  set (MEXLIB_LDFLAGS "${MEXLIB_LDFLAGS}" PARENT_SCOPE)
  # todo: add CLIBS or CXXLIBS to LINK_FLAGS selectively based on if it's a c or cxx target (always added CXX above)

  if (MATLAB_ROOT AND MEX_EXT)
     set (MATLAB_FOUND true PARENT_SCOPE)
     message(STATUS "Found matlab")
  endif()

  compare_compilers(compilers_match "${CMAKE_C_COMPILER}" "${MEX_CC}")
  if (NOT compilers_match)
     message(WARNING "Your cmake C compiler is: \"${CMAKE_C_COMPILER}\" but your mex options use: \"${MEX_CC}\" (the compiler version strings are printed above).  You must use the same compilers.  You can either:\n  a) reconfigure the mex compiler by running 'mex -setup' in  MATLAB, or\n  b) Set the default compiler for cmake by setting the CC environment variable in your terminal.\n")
  endif()

  compare_compilers(compilers_match "${CMAKE_CXX_COMPILER}" "${MEX_CXX}")
  if (NOT compilers_match)
     message(WARNING "Your cmake CXX compiler is: \"${CMAKE_CXX_COMPILER}\" but your mex options end up pointing to: \"${MEX_CXX}\".  You must use the same compilers.  You can either:\n  a) Configure the mex compiler by running 'mex -setup' in  MATLAB, or \n  b) Set the default compiler for cmake by setting the CXX environment variable in your terminal.")
  endif()

  # NOTE:  would like to check LD also, but it appears to be difficult with cmake  (there is not explicit linker executable variable, only the make rule), and  even my mex code assumes that LD==LDCXX for simplicity.

endfunction()

function(add_mex)
  # useage:  add_mex(target source1 source2 [SHARED,EXECUTABLE] [OUTPUT_NAME mexfilename])
  # note: builds the mex file inplace (not into some build directory)
  # if SHARED is passed in, then it doesn't expect a mexFunction symbol to be defined, and compiles it to e.g., libtarget.so, for eventual linking against another mex file
  # if EXECUTABLE is passed in, then it adds an executable target, which is linked against the appropriate matlab libraries.

  list(GET ARGV 0 target)
  list(REMOVE_AT ARGV 0)

  if (NOT MATLAB_FOUND)
    message(STATUS "${target} will be skipped because matlab was not found")
    return() # return quietly if matlab is not found.  (note: if matlab/mex is REQUIRED, then it will have been found before getting here)
  endif()

  include_directories( ${MATLAB_ROOT}/extern/include ${MATLAB_ROOT}/simulink/include )

  list(FIND ARGV OUTPUT_NAME output_arg)
  if (output_arg GREATER -1)
    list(REMOVE_AT ARGV ${output_arg})
    list(GET ARGV ${output_arg} mexfilename)
    list(REMOVE_AT ARGV ${output_arg})
  else()
    set(mexfilename ${target})
  endif()

  list(FIND ARGV SHARED isshared)
  list(FIND ARGV EXECUTABLE isexe)

  # Set compiler and linker flags for MEX file compilation. Be careful not to
  # overwrite these COMPILE_FLAGS when setting additional properties on MEX
  # targets to suppress specific compiler warnings, etc.
  if (isexe GREATER -1)
    list(REMOVE_ITEM ARGV EXECUTABLE)
    add_executable(${target} ${ARGV})
    set_target_properties(${target} PROPERTIES
      COMPILE_FLAGS "${MEX_COMPILE_FLAGS}"
      OUTPUT_NAME ${mexfilename})
#      INSTALL_RPATH "${CMAKE_INSTALL_PATH};${MEX_RPATH}")
    if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
      target_link_libraries(${target} liblast)
    else()
      set_target_properties(${target} PROPERTIES LINK_FLAGS "${MEXLIB_LDFLAGS}")
    endif()
  elseif (isshared GREATER -1)
    add_library(${target} ${ARGV})
    set_target_properties(${target} PROPERTIES
      COMPILE_FLAGS "${MEX_COMPILE_FLAGS}"
      OUTPUT_NAME ${mexfilename}
      CXX_VISIBILITY_PRESET "default"
      VISIBILITY_INLINES_HIDDEN 0)
    if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
      target_link_libraries(${target} liblast)
    else()
      string(REPLACE "/export:mexFunction" "" __ldflags "${MEXLIB_LDFLAGS}")
      string(REPLACE "/EXPORT:mexFunction" "" __ldflags "${__ldflags}")
      string(REGEX REPLACE "/implib:[^ ]+" "" __ldflags "${__ldflags}")

      set_target_properties(${target} PROPERTIES LINK_FLAGS "${__ldflags}")
    endif()
  else ()
    add_library(${target} MODULE ${ARGV})
    set_target_properties(${target} PROPERTIES
      COMPILE_FLAGS "${MEX_COMPILE_FLAGS}"
      PREFIX ""
      SUFFIX ".${MEX_EXT}"
      LINK_FLAGS "${MEXLIB_LDFLAGS}"
      ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      OUTPUT_NAME ${mexfilename}
      CXX_VISIBILITY_PRESET "default"
      VISIBILITY_INLINES_HIDDEN 0
      )
    foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
      string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
      set_target_properties(${target} PROPERTIES
	      ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_CURRENT_SOURCE_DIR}"
        	LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_CURRENT_SOURCE_DIR}"
	      )
    endforeach()

    if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
      # see comment by the definition of liblast above
      set_target_properties(${target} PROPERTIES LINK_FLAGS "${MEXLIB_LDFLAGS}")
      target_link_libraries(${target} liblast)
    endif()
  endif()

endfunction()

function(get_compiler_version outvar compiler)
  if ( MSVC )
    execute_process(
      COMMAND "${compiler}"
      ERROR_VARIABLE output
      ERROR_STRIP_TRAILING_WHITESPACE
      OUTPUT_VARIABLE output
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCH "([0-9]+\\.[0-9]+\\.[0-9]+)" ver "${output}")
  else()
    string(REPLACE "\"" "" compiler ${compiler}) # remove quotes
    separate_arguments(c_args UNIX_COMMAND ${compiler})
    list(APPEND c_args "-dumpversion")
    execute_process(COMMAND ${c_args} OUTPUT_VARIABLE ver OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif()
  set(${outvar} ${ver} PARENT_SCOPE)

endfunction()

## calls compilers with --version option and checks the output
# calls compilers with -dumpversion and checks the output
# (because it appears that ccache messes with the --version output)
# returns TRUE if the strings match or FALSE if they don't.
#   (note: you can use  if (outvar) to test )
# this seems to be a more robust and less complex method than trying to call xcrun -find, readlink to follow symlinks, etc.
function(compare_compilers outvar compiler1 compiler2)

  get_compiler_version(c1_ver ${compiler1})
  get_compiler_version(c2_ver ${compiler2})

  if (c1_ver AND c2_ver AND "${c1_ver}" STREQUAL "${c2_ver}")
    set(${outvar} TRUE PARENT_SCOPE)
  else()
    set(${outvar} FALSE PARENT_SCOPE)
    message(STATUS "compiler1 version string:\n${c1_ver}")
    message(STATUS "compiler2 version string:\n${c2_ver}")
  endif()

endfunction()

include(CMakeParseArguments)
