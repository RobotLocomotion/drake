#!/bin/bash

echo "$@" > /tmp/args

exec "$(which protoc)" "$@"
