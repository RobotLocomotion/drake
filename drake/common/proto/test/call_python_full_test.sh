#!/bin/bash
set -e -u

# @file
# @brief Tests the `call_python_client` CLI and `call_python` test together.

cc_bin_flags=
while [[ $# -gt 2 ]]; do
    case ${1} in
        --no_plotting)
            cc_bin_flags='--gtest_filter=-TestCallPython.Plot*'
            shift;;
        *)
            echo "Bad argument: ${1}" >&2
            exit 1;;
    esac
done

cc_bin=${1}
py_client_cli=${2}

if [[ ! -e /tmp/python_rpc ]]; then
    mkfifo /tmp/python_rpc
fi

# Start Python binary in the background.
${py_client_cli} --no_loop &
pid=$!
# Execute C++.
${cc_bin} ${cc_bin_flags}
# When this is done, Python client should exit.
wait $! || { echo "ERROR: Python client did not exit successfully."; exit 1; }
