## This file should be placed in the root directory of your project.
## Then modify the CMakeLists.txt file in the root directory of your
## project to incorporate the testing dashboard.
## # The following are required to uses Dart and the Cdash dashboard
##   ENABLE_TESTING()
##   INCLUDE(CTest)
set(CTEST_PROJECT_NAME "DRC")
set(CTEST_NIGHTLY_START_TIME "22:00:00 EDT")

set(CTEST_DROP_METHOD "http")
set(CTEST_DROP_SITE "128.30.31.107")
set(CTEST_DROP_LOCATION "/cdash/submit.php?project=DRC")
set(CTEST_DROP_SITE_CDASH TRUE)
