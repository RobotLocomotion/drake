#!/bin/bash

# This wrapper script calls "protoc" on the default $PATH, avoiding a need to
# declare a dependency on it.

exec "protoc" "$@"
