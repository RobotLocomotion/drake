#!/bin/sh
# Utility script to rewrite all object file symbols to be local, not global.

object="$1"
if [ ! -w "${object}" ]; then
    echo "error: Cannot read/write file '${object}'" 1>&2
    exit 1
fi

/usr/bin/objcopy -G nothing_will_remain_global "${object}"
