#!/bin/sh
glxinfo
Xvfb :88 -ac -screen 0 1280x1024x24 &
DISPLAY=:88 $1
DISPLAY=:88 glxinfo
xvfb_pid="`pgrep -f "Xvfb :88"`"
kill -9 ${xvfb_pid}
# xvfb-run -a -s "-screen 0 1280x1024x24" $1
