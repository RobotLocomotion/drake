# This directory contains black-box regression tests that operate on the
# installed outputs of the Drake build.

# Xvfb is a virtual framebuffer for X11, typically only available on Linux. We
# attempt to use it here because a display is not always available, and to
# prevent window appearing on-screen during the test.  On other supported
# platforms (Windows and OS X), a native display is always available, and
# always used.
find_program(xvfb-run xvfb-run)

set(haveXvfb FALSE)
if(xvfb-run)
  execute_process(
    COMMAND xvfb-run -s "-screen 0 1024x768x24" glxinfo -B
    COMMAND grep -i glx
    RESULT_VARIABLE xvfbGlxReturnCode
    OUTPUT_QUIET
    ERROR_QUIET
    )
  if(xvfbGlxReturnCode EQUAL 0)
    set(haveXvfb TRUE)
  endif()
endif()

set(haveDisplay FALSE)
if(APPLE OR WIN32)
  set(haveDisplay TRUE)
elseif(DEFINED ENV{DISPLAY})
  set(haveDisplay TRUE)
endif()

set(displayMethod "none")
if(APPLE OR WIN32)
  # Never use xvfb; always use display.
  set(displayMethod "primaryDisplay")
else()
  # On linux: prefer xvfb, fall back to primary display.
  if(haveXvfb)
    set(displayMethod "xvfbDisplay")
  elseif(haveDisplay)
    set(displayMethod "primaryDisplay")
  endif()
endif()

if(WITH_DIRECTOR)
  # A test that starts up drake-visualizer.
  set(visualizerStartupTestCommand
      ${CMAKE_INSTALL_PREFIX}/bin/directorPython
      ${CMAKE_CURRENT_SOURCE_DIR}/visualizer_startup_test.py)
  if(displayMethod STREQUAL "primaryDisplay")
    drake_add_test(NAME visualizer_startup_test_default_display
             COMMAND ${visualizerStartupTestCommand}
             SIZE medium)
  elseif(displayMethod STREQUAL "xvfbDisplay")
    drake_add_test(NAME visualizer_startup_test_xvfb
             COMMAND xvfb-run -s "-screen 0 1024x768x24"
             ${visualizerStartupTestCommand}
             SIZE medium)
  else()
    message(STATUS "visualizer_startup_test is disabled"
      " (no display and xvfb-run is either absent or does not support GLX)")
  endif()
endif()

if(MATLAB_FOUND)
  drake_add_matlab_test(NAME addpath_spotless_test COMMAND "addpath_spotless;")
endif()
