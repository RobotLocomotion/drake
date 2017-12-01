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

py-error() {
    echo "ERROR: Python client did not exit successfully."
    exit 1
}

should-fail() {
    echo "This should have failed!"
    exit 2
}

sub-tests() {
    # Execute sub-cases.
    func=${1}
    # Sub-case 1: Nominal
    # @note This setup assumes other things succeeded.
    echo -e "\n[ ${func}: nominal ]"
    set-flags 0 0
    ${func}
    # Sub-case 2: With Error
    echo -e "\n[ ${func}: with_error ]"
    set-flags 1 0
    ${func}
    # Sub-case 3: With Error + Stop on Error
    echo -e "\n[ ${func}: with_error + stop_on_error ]"
    set-flags 1 1
    ${func}
}

py-check() {
    # Check result of Python process.
    if [[ ${py_fail} -eq 0 ]]; then
        # Should succeed.
        wait ${pid} || py-error
    else
        # Should fail.
        # TODO(eric.cousineau): File / find bug in Bash for this; this behaves
        # differently depending on how this is placed in a function.
        { wait ${pid} && should-fail; } || :
    fi
}

set-flags() {
    py_fail=${1}
    py_stop_on_error=${2}

    cc_flags=
    if [[ ${py_fail} -eq 1 ]]; then
        cc_flags='--with_error'
    fi
    py_flags=
    if [[ ${py_stop_on_error} -eq 1 ]]; then
        py_flags='--stop_on_error'
    fi
}

# Execute tests.

no_threading-no_loop() {
    # Start Python binary in the background.
    ${py_client_cli} --no_threading --no_loop ${py_flags} &
    pid=$!
    # Execute C++.
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    # When this is done, Python client should exit.
    py-check
}
sub-tests no_threading-no_loop

threading-no_loop() {
    ${py_client_cli} --no_loop ${py_flags} &
    pid=$!
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    py-check
}
sub-tests threading-no_loop

threading-loop() {
    # Use `exec` so that we inherit SIGINT handlers.
    # @ref https://stackoverflow.com/a/44538799/7829525
    exec ${py_client_cli} ${py_flags} &
    pid=$!
    # Execute client twice.
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    if [[ ${py_stop_on_error} -ne 1 ]]; then
        # Wait for a small bit, then kill the client with Ctrl+C.
        # This is necessary to permit the client to finish processing.
        sleep 0.2
        # TODO(eric.cousineau): In script form, this works well (only one
        # interrupt needed); however, interactively we need a few more.
        kill -INT ${pid} || :
    fi
    py-check
}
sub-tests threading-loop
