#!/bin/bash

valgrind \
    --leak-check=full \
    --suppressions="tools/valgrind.supp" \
    --error-exitcode=1 \
    "$@"
