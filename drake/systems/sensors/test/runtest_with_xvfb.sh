#!/bin/sh
lspci | grep VGA
ldd $1
libgl="`ldd $1 | grep libGL.so | grep nvidia`"
if [ ! -z "$libgl" ]
then
  $1
else
  Xvfb :88 -ac -screen 0 1280x1024x24 &
  sleep 1
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
