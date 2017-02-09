include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)
include(CheckFortranCompilerFlag)

set(MEMORYCHECK_ERROR_EXITCODE 9999)

#------------------------------------------------------------------------------
# Set compiler flags and CTest variables for dynamic and static analysis tools
# for the drake-superbuild project
#------------------------------------------------------------------------------
macro(drake_setup_tools_superbuild)
  set(_tools_dir "${CMAKE_CURRENT_LIST_DIR}/tools")
  drake_setup_tools_common()
  unset(_tools_dir)
endmacro()

#------------------------------------------------------------------------------
# Set compiler flags and CTest variables for dynamic and static analysis tools
# for the drake project
#------------------------------------------------------------------------------
macro(drake_setup_tools)
  get_filename_component(_tools_dir "${CMAKE_CURRENT_LIST_DIR}" DIRECTORY)
  set(_tools_dir "${_tools_dir}/tools")
  drake_setup_tools_common()
  unset(_tools_dir)
endmacro()

#------------------------------------------------------------------------------
# Set compiler flags and CTest variables for dynamic and static analysis tools
#------------------------------------------------------------------------------
macro(drake_setup_tools_common)
  set(TESTS_ENVIRONMENT)
  drake_setup_clang_tidy()
  drake_setup_coverage()
  drake_setup_include_what_you_use()
  drake_setup_link_what_you_use()
  drake_setup_sanitizers()
  drake_setup_valgrind()
endmacro()

#------------------------------------------------------------------------------
# Add a compiler flag for all languages if the compiler for that language
# supports the flag.
#------------------------------------------------------------------------------
macro(_drake_add_compiler_flag_if_supported FLAG NAME)
  check_c_compiler_flag("-Werror ${FLAG}" C_COMPILER_SUPPORTS_${NAME})

  if(C_COMPILER_SUPPORTS_${NAME})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAG}")
  endif()

  check_cxx_compiler_flag("-Werror ${FLAG}" CXX_COMPILER_SUPPORTS_${NAME})

  if(CXX_COMPILER_SUPPORTS_${NAME})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")
  endif()


  if(CMAKE_Fortran_COMPILER AND NOT DISABLE_FORTRAN)
    check_fortran_compiler_flag("-Werror ${FLAG}" Fortran_COMPILER_SUPPORTS_${NAME})

    if(Fortran_COMPILER_SUPPORTS_${NAME})
      set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} ${FLAG}")
    endif()
  endif()
endmacro()

#------------------------------------------------------------------------------
# Add a compiler and linker flag for all languages if the compiler for that
# language supports the flag. Error if the C or C++ compiler does not support
# the flag, but disable Fortran if the Fortran compiler does not support the
# flag.
#------------------------------------------------------------------------------
macro(_drake_add_compiler_and_linker_flag FLAG NAME)
  set(SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
  # Temporarily use a different value for CMAKE_REQUIRED_LIBRARIES for the call
  # to check_cxx_source_compiles() via check_cxx_compiler_flag().
  set(CMAKE_REQUIRED_LIBRARIES ${FLAG})
  _drake_add_compiler_flag_if_supported("${FLAG}" ${NAME})
  # Restore the previous value as CMAKE_REQUIRED_LIBRARIES is used by other
  # modules in CMake.
  set(CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})

  if(NOT CXX_COMPILER_SUPPORTS_${NAME})
    message(FATAL_ERROR "C++ compiler does not support ${FLAG}")
  endif()

  if(NOT C_COMPILER_SUPPORTS_${NAME})
    message(FATAL_ERROR "C compiler does not support ${FLAG}")
  endif()

  if(CMAKE_Fortran_COMPILER AND NOT DISABLE_FORTRAN AND NOT Fortran_COMPILER_SUPPORTS_${NAME})
    message(STATUS
      "Disabling Fortran because compiler does not support ${FLAG}")
    set(DISABLE_FORTRAN ON)
  endif()

  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${FLAG}")
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${FLAG}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${FLAG}")
endmacro()

#------------------------------------------------------------------------------
# Set common compiler flags for sanitizers and Valgrind tools
#------------------------------------------------------------------------------
macro(drake_add_common_memcheck_flags)
  _drake_add_compiler_flag_if_supported("-fno-omit-frame-pointer"
    FNO_OMIT_FRAME_POINTER)

  if(NOT CMAKE_CONFIGURATION_TYPES)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      _drake_add_compiler_flag_if_supported("-O1" O1)
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "MinSizeRel" OR CMAKE_BUILD_TYPE STREQUAL "Release")
      _drake_add_compiler_flag_if_supported("-gline-tables-only"
        GLINE_TABLES_ONLY)
    endif()
  endif()

  set(ENV{GTEST_DEATH_TEST_USE_FORK} 1)
endmacro()

#------------------------------------------------------------------------------
# Set CMake variables for clang-tidy. Requires CMake 3.6 or above.
#------------------------------------------------------------------------------
macro(drake_setup_clang_tidy)
  if(CMAKE_VERSION VERSION_EQUAL 3.6 OR CMAKE_VERSION VERSION_GREATER 3.6)
    if(CMAKE_GENERATOR STREQUAL "Ninja" OR CMAKE_GENERATOR STREQUAL "Unix Makefiles")
      find_program(CLANG_TIDY_EXECUTABLE NAMES clang-tidy)
      mark_as_advanced(CLANG_TIDY_EXECUTABLE)

      if(CLANG_TIDY_EXECUTABLE)
        option(USE_CLANG_TIDY "Use clang-tidy" OFF)

        if(USE_CLANG_TIDY)
          set(CMAKE_C_CLANG_TIDY "${CLANG_TIDY_EXECUTABLE}")
          set(CMAKE_CXX_CLANG_TIDY ${CMAKE_C_CLANG_TIDY})
        endif()
      endif()
    endif()
  endif()
endmacro()

#------------------------------------------------------------------------------
# Set CMake variables for include-what-you-use
#------------------------------------------------------------------------------
macro(drake_setup_include_what_you_use)
  if(CMAKE_GENERATOR STREQUAL "Ninja" OR CMAKE_GENERATOR STREQUAL "Unix Makefiles")
    find_program(INCLUDE_WHAT_YOU_USE_EXECUTABLE NAMES include-what-you-use)
    mark_as_advanced(INCLUDE_WHAT_YOU_USE_EXECUTABLE)

    if(INCLUDE_WHAT_YOU_USE_EXECUTABLE)
      option(USE_INCLUDE_WHAT_YOU_USE "Use include-what-you-use" OFF)

      if(USE_INCLUDE_WHAT_YOU_USE)
        set(CMAKE_C_INCLUDE_WHAT_YOU_USE
          "${INCLUDE_WHAT_YOU_USE_EXECUTABLE}"
          "-Xiwyu"
          "--mapping_file=${_tools_dir}/include-what-you-use.imp")
        set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${CMAKE_C_INCLUDE_WHAT_YOU_USE})
      endif()
    endif()
  endif()
endmacro()

#------------------------------------------------------------------------------
# Set CMake variables for link-what-you-use
#------------------------------------------------------------------------------
macro(drake_setup_link_what_you_use)
  if(CMAKE_VERSION VERSION_EQUAL 3.7 OR CMAKE_VERSION VERSION_GREATER 3.7)
    find_program(LDD_EXECUTABLE NAMES ldd)
    mark_as_advanced(LDD_EXECUTABLE)

    if(LDD_EXECUTABLE)
      option(USE_LINK_WHAT_YOU_USE
        "Run ldd -r -u on targets after they are linked" OFF)

      if(USE_LINK_WHAT_YOU_USE)
        set(CMAKE_LINK_WHAT_YOU_USE ON)
      endif()
    endif()
  endif()
endmacro()

#------------------------------------------------------------------------------
# Set compiler and linker flags and CTest variables for sanitizers
#------------------------------------------------------------------------------
macro(drake_setup_sanitizers)
  set(USE_SANITIZER OFF CACHE STRING
    "Choose a sanitizer to use; options are Address Leak Memory MemoryWithOrigins Thread Undefined")
  set_property(CACHE USE_SANITIZER PROPERTY STRINGS
    OFF Address Leak Memory MemoryWithOrigins Thread Undefined)

  if(USE_SANITIZER)
    if(NOT DISABLE_MATLAB)
      set(DISABLE_MATLAB ON)
      message(STATUS "Disabling MATLAB because USE_SANITIZER is enabled")
    endif()

    if(NOT DISABLE_PYTHON)
      set(DISABLE_PYTHON ON)
      message(STATUS "Disabling Python because USE_SANITIZER is enabled")
    endif()

    drake_add_common_memcheck_flags()

    set(SANITIZER_COMMON_OPTIONS "exitcode=${MEMORYCHECK_ERROR_EXITCODE}")

    if(USE_SANITIZER STREQUAL "Address")
      _drake_add_compiler_and_linker_flag("-fsanitize=address" FSANITIZE_ADDRESS)
      set(MEMORYCHECK_TYPE AddressSanitizer)
      set(ENV{ASAN_OPTIONS}
        "${SANITIZER_COMMON_OPTIONS}:suppressions=${_tools_dir}/asan.supp:$ENV{ASAN_OPTIONS}")
      list(APPEND TESTS_ENVIRONMENT ASAN_OPTIONS=$ENV{ASAN_OPTIONS})
    elseif(USE_SANITIZER STREQUAL "Leak")
      _drake_add_compiler_and_linker_flag("-fsanitize=leak" FSANITIZE_LEAK)
      # LeakSanitizer is part of AddressSanitizer
      set(MEMORYCHECK_TYPE AddressSanitizer)
      set(ENV{LSAN_OPTIONS}
        "${SANITIZER_COMMON_OPTIONS}:suppressions=${_tools_dir}/lsan.supp:$ENV{LSAN_OPTIONS}")
      list(APPEND TESTS_ENVIRONMENT LSAN_OPTIONS=$ENV{LSAN_OPTIONS})
    elseif(USE_SANITIZER MATCHES "^Memory(WithOrigins)?$")

      # Work around the generator not supporting Fortran for the compiler
      # checks.
      if(NOT CMAKE_GENERATOR STREQUAL "Unix Makefiles")
        set(DISABLE_FORTRAN ON)
      endif()

      _drake_add_compiler_and_linker_flag("-fsanitize=memory" FSANITIZE_MEMORY)

      if(USE_SANITIZER STREQUAL "MemoryWithOrigins")
        _drake_add_compiler_and_linker_flag("-fsanitize-memory-track-origins"
          FSANITIZE_MEMORY_TRACK_ORIGINS)
      endif()

      set(MEMORYCHECK_TYPE MemorySanitizer)
      set(ENV{MSAN_OPTIONS}
        "${SANITIZER_COMMON_OPTIONS}:suppressions=${MSAN_OPTIONS}/msan.supp:$ENV{MSAN_OPTIONS}")
      list(APPEND TESTS_ENVIRONMENT MSAN_OPTIONS=$ENV{MSAN_OPTIONS})
    elseif(USE_SANITIZER STREQUAL "Thread")
      _drake_add_compiler_and_linker_flag("-fsanitize=thread" FSANITIZE_THREAD)
      set(MEMORYCHECK_TYPE ThreadSanitizer)
      set(ENV{TSAN_OPTIONS}
        "${SANITIZER_COMMON_OPTIONS}:suppressions=${_tools_dir}/tsan.supp:$ENV{TSAN_OPTIONS}")
      list(APPEND TESTS_ENVIRONMENT TSAN_OPTIONS=$ENV{TSAN_OPTIONS})
    elseif(USE_SANITIZER STREQUAL "Undefined")
      _drake_add_compiler_and_linker_flag("-fsanitize=undefined"
        FSANITIZE_UNDEFINED)
      set(MEMORYCHECK_TYPE UndefinedBehaviorSanitizer)
      set(ENV{UBSAN_OPTIONS}
        "${SANITIZER_COMMON_OPTIONS}:suppressions=${_tools_dir}/ubsan.supp:$ENV{UBSAN_OPTIONS}")
      list(APPEND TESTS_ENVIRONMENT UBSAN_OPTIONS=$ENV{UBSAN_OPTIONS})
    else()
      message(FATAL_ERROR "Sanitizer ${USE_SANITIZER} is NOT supported")
    endif()
  endif()
endmacro()

#------------------------------------------------------------------------------
# Set compiler and linker flags and CTest variables for Valgrind tools
#------------------------------------------------------------------------------
macro(drake_setup_valgrind)
  find_program(VALGRIND_EXECUTABLE NAMES valgrind)
  mark_as_advanced(VALGRIND_EXECUTABLE)

  if(VALGRIND_EXECUTABLE)
    set(USE_VALGRIND OFF CACHE STRING
      "Choose a Valgrind tool to use; options are Cachegrind Callgrind DRD Hellgrind Massif Memcheck")
    set_property(CACHE USE_VALGRIND PROPERTY STRINGS
      OFF Cachegrind Callgrind DRD Hellgrind Massif Memcheck)

    if(USE_VALGRIND)
      if(NOT DISABLE_MATLAB)
        set(DISABLE_MATLAB ON)
        message(STATUS "Disabling MATLAB because USE_VALGRIND is enabled")
      endif()

      if(NOT DISABLE_PYTHON)
        set(DISABLE_PYTHON ON)
        message(STATUS "Disabling Python because USE_VALGRIND is enabled")
      endif()

      string(TOLOWER "${USE_VALGRIND}" _use_valgrind_lower)

      drake_add_common_memcheck_flags()

      if(_use_valgrind_lower MATCHES "(cachegrind|callgrind|drd|hellgrind|massif|memcheck)")
        set(MEMORYCHECK_TYPE Valgrind)

        set(VALGRIND_COMMAND "${VALGRIND_EXECUTABLE}")
        set(VALGRIND_COMMAND_OPTIONS
          "--error-exitcode=${MEMORYCHECK_ERROR_EXITCODE} --tool=${_use_valgrind_lower} --trace-children=yes --trace-children-skip=/bin/*,/usr/bin/*,/usr/local/bin/* --track-origins=yes")

        if(_use_valgrind_lower STREQUAL "memcheck")
          set(VALGRIND_COMMAND_OPTIONS
            "${VALGRIND_COMMAND_OPTIONS} --leak-check=full --show-leak-kinds=definite,possible")
        endif()

        # TODO(jamiesnape): Use common Valgrind suppressions for Bazel and CMake.
        set(MEMORYCHECK_SUPPRESSIONS_FILE "${_tools_dir}/valgrind-cmake.supp")
      endif()

      unset(_use_valgrind_lower)
    endif()
  endif()
endmacro()

#------------------------------------------------------------------------------
macro(drake_setup_coverage)
  set(USE_COVERAGE OFF CACHE STRING
    "Choose a coverage tool to use; options are Gcov Sanitizer")
  set_property(CACHE USE_COVERAGE PROPERTY STRINGS OFF Gcov Sanitizer)

  if(USE_COVERAGE)
    if(NOT DISABLE_PYTHON)
      set(DISABLE_PYTHON ON)
      message(STATUS "Disabling Python because USE_COVERAGE is enabled")
    endif()

    if(USE_COVERAGE STREQUAL "Gcov")
      _drake_add_compiler_and_linker_flag("-fprofile-arcs" FPROFILE_ARCS)
      _drake_add_compiler_and_linker_flag("-ftest-coverage" FTEST_COVERAGE)

      find_program(LCOV_EXECUTABLE lcov)
      mark_as_advanced(LCOV_EXECUTABLE)

      find_program(GENHTML_EXECUTABLE genhtml)
      mark_as_advanced(GENHTML_EXECUTABLE)

      if(LCOV_EXECUTABLE AND GENHTML_EXECUTABLE)
        add_custom_target(lcov
          COMMAND "${LCOV_EXECUTABLE}"
            --capture
            --directory .
            --output-file "${CMAKE_CURRENT_BINARY_DIR}/coverage.info"
          COMMAND "${GENHTML_EXECUTABLE}"
            "${CMAKE_CURRENT_BINARY_DIR}/coverage.info"
            --output-directory "${CMAKE_CURRENT_BINARY_DIR}/coverage-html")
      else()
        message(STATUS
          "Could NOT find lcov and/or genhtml. HTML coverage report will NOT be created.")
      endif()

    elseif(USE_COVERAGE STREQUAL "Sanitizer")
      if(NOT DISABLE_MATLAB)
        set(DISABLE_MATLAB ON)
        message(STATUS "Disabling MATLAB because USE_COVERAGE=Sanitizer is enabled")
      endif()

      set(SANITIZER_COVERAGE_LEVEL Edge CACHE STRING
        "Choose a sanitizer coverage level to use; options are BasicBlock Edge Source")
      set_property(CACHE SANITIZER_COVERAGE_LEVEL PROPERTY STRINGS
        BasicBlock Edge Function)

      if(SANITIZER_COVERAGE_LEVEL STREQUAL "BasicBlock")
         _drake_add_compiler_and_linker_flag("-fsanitize-coverage=bb"
           FSANITIZE_COVERAGE_BB)
      elseif(SANITIZER_COVERAGE_LEVEL STREQUAL "Edge")
         _drake_add_compiler_and_linker_flag("-fsanitize-coverage=edge"
           FSANITIZE_COVERAGE_EDGE)
      elseif(SANITIZER_COVERAGE_LEVEL STREQUAL "Function")
         _drake_add_compiler_and_linker_flag("-fsanitize-coverage=func"
           FSANITIZE_COVERAGE_FUNC)
      else()
        message(FATAL_ERROR
          "Sanitizer coverage level ${SANITIZER_COVERAGE_LEVEL} is NOT supported")
      endif()

      set(ENV{UBSAN_OPTIONS} "coverage=1:$ENV{UBSAN_OPTIONS}")

      find_program(SANCOV_EXECUTABLE NAMES sancov)
      mark_as_advanced(SANCOV_EXECUTABLE)

      if(SANCOV_EXECUTABLE)
        set(ENV{UBSAN_OPTIONS}
          "html_cov_report=1:sancov_path=${SANCOV_EXECUTABLE}:$ENV{UBSAN_OPTIONS}")
      else()
        message(STATUS "Could NOT find sancov. HTML coverage report will NOT be created.")
      endif()

      list(APPEND TESTS_ENVIRONMENT UBSAN_OPTIONS=$ENV{UBSAN_OPTIONS})
    else()
      message(FATAL_ERROR "Coverage tool ${USE_COVERAGE} is not supported")
    endif()
  endif()
endmacro()
