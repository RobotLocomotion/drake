include(HOSTNAME_common.cmake)
set(CTEST_BUILD_NAME ubuntu-12.04-x64-gcc-4.6.3-trunk)
set(dashboard_model Continuous)
set(dashboard_svn_branch trunk)
set(dashboard_skip_testing 1)

include(${CTEST_SCRIPT_DIRECTORY}/dashboard_common.cmake)
