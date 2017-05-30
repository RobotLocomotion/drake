#!/bin/bash

if [ -z "$DISPLAY" ] && type -P xvfb-run; then
    exec xvfb-run --auto-servernum --server-args="-screen 0 1024x768x24" "$@"
else
    exec "$@"
fi
