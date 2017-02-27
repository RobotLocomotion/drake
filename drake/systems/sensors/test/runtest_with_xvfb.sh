#!/bin/sh
lspci | grep VGA
ldd $1
Xvfb :88 -ac -screen 0 1280x1024x24 &
# Xorg -noreset +extension GLX +extension RANDR +extension RENDER -config ./xorg.conf :88 &
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
