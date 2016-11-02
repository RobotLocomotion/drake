# This directory contains black-box regression tests that operate on the
# installed outputs of the Drake build.

# Xvfb is a virtual framebuffer for Linux. We use it if no native display is
# available. On other supported platforms (Windows and OS X), a native display
# is always available.
find_program(xvfb-run xvfb-run)

if(APPLE OR WIN32 OR DEFINED ENV{DISPLAY})
  set(hasDisplay TRUE)
else()
  set(hasDisplay FALSE)
endif()

# TODO(patmarion)
# The visualization test is currently disabled due to a recent change in
# dependencies.  Now director depends on drake, so the drake-visualizer
# program is compiled after drake.  The ctest scripts running continuous
# integration will be updated to run director tests which will cover the
# visualizer.  Afterward, drake developers will decide whether we remove
# this test here or add new support for running tests after superbuild.
if(FALSE AND WITH_DIRECTOR)
  # A test that starts up drake-visualizer.
  set(visualizerStartupTestCommand
      ${CMAKE_INSTALL_PREFIX}/bin/directorPython
      ${CMAKE_CURRENT_SOURCE_DIR}/visualizer_startup_test.py)
  if(hasDisplay)
    drake_add_test(NAME visualizer_startup_test_default_display
             COMMAND ${visualizerStartupTestCommand}
             SIZE medium)
  elseif(xvfb-run)
    drake_add_test(NAME visualizer_startup_test_xvfb
             COMMAND xvfb-run -s "-screen 0 1024x768x24"
             ${visualizerStartupTestCommand}
             SIZE medium)
  endif()
endif()
