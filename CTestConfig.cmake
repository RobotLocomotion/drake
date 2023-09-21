# -*- mode: cmake -*-
# vi: set ft=cmake :

set(CTEST_PROJECT_NAME drake)
set(CTEST_NIGHTLY_START_TIME "00:00:00 EST")
set(CTEST_DROP_METHOD http)
set(CTEST_DROP_SITE ec2-34-203-13-209.compute-1.amazonaws.com/cdash)
set(CTEST_DROP_LOCATION "/submit.php?project=${CTEST_PROJECT_NAME}")
set(CTEST_DROP_SITE_CDASH ON)
