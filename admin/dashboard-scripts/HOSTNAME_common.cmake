set(CTEST_SITE "paladin-02.csail.mit.edu")
set(CTEST_BUILD_CONFIGURATION Release)
set(CTEST_CMAKE_GENERATOR "Unix Makefiles")
set(CTEST_BUILD_FLAGS "-j4")
set(CTEST_TEST_TIMEOUT 7200)
set(CTEST_DASHBOARD_ROOT "${CTEST_SCRIPT_DIRECTORY}")
set(ENV{DISPLAY} ":0")


# valgrind and gcov
set(dashboard_do_memcheck FALSE)
set(dashboard_do_coverage FALSE)
set(CTEST_MEMORYCHECK_COMMAND "/usr/bin/valgrind")
set(CTEST_MEMORYCHECK_SUPPRESSIONS_FILE "${CTEST_SCRIPT_DIRECTORY}/drc001_valgrind_supp.txt")
set(CTEST_MEMORYCHECK_COMMAND_OPTIONS "-q --leak-check=full --num-callers=50")
#set(CTEST_MEMORYCHECK_COMMAND_OPTIONS "--gen-suppressions=all --leak-check=full --show-reachable=yes --num-callers=50")
set(CTEST_COVERAGE_COMMAND "/usr/bin/gcov")


set(ENV{CXXFLAGS} "-Wall -Wextra")
set(ENV{CFLAGS} "-Wall -Wextra")

if(dashboard_do_coverage)
  set(ENV{CXXFLAGS} $ENV{CXXFLAGS} -O0 --coverage)
  set(ENV{CFLAGS} $ENV{CFLAGS} -O0 --coverage)
  set(ENV{LDFLAGS} $ENV{LDFLAGS} --coverage)
endif()

list(APPEND CTEST_NOTES_FILES
  "${CMAKE_CURRENT_LIST_FILE}"
  )
