#!/bin/sh
# Runs a ctest executable which is passed as the first argument.  If libGL.so is
# not the one from Nvidia, Xvfb is executed before calling the test executable.
# Otherwise, Xvfb is not called.  This script assumes that Xvfb is installed.
#
# @param $1 A ctest executable.
lspci | grep VGA
ldd $1
libgl="`ldd $1 | grep libGL.so | grep nvidia`"
if [ ! -z "$libgl" ]
then
  $1
else
  Xvfb :88 -ac -screen 0 1280x1024x24 &
  sleep 3
  DISPLAY=:88 $1
  test_result=$?
  if [ "$test_result" == "0" ]
    then
    echo "PASS"
  else
    echo "FAIL"
  fi
  xvfb_pid="`pgrep -f "Xvfb :88"`"
  kill -9 $xvfb_pid
fi
