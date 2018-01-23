#!/bin/bash
set -e -u

# @file
# @brief Tests the `call_python_client` CLI and `call_python` test together.

# TODO(eric.cousineau): Rewrite this in Python for an easier-to-understand
# testing API (#7703).

no_plotting=
# By default, set backend so that the test does not open windows.
export MPLBACKEND="ps"

while [[ $# -gt 0 ]]; do
    case ${1} in
        --no_plotting)
            no_plotting=1
            shift;;
        --matplotlib_backend)
            export MPLBACKEND=${2}
            shift; shift;;
        *)
            echo "Bad argument: ${1}" >&2
            exit 1;;
    esac
done

cur=$(dirname $0)
cc_bin=${cur}/call_python_test
py_client_cli=${cur}/call_python_client_cli
# TODO(eric.cousineau): Use `tempfile` once we can choose which file C++
# uses.
filename=$(mktemp)
done_file=${filename}_done

is_mac=
if [[ "${OSTYPE}" == "darwin"* ]]; then
    is_mac=1
    # Do not test if Mac has matplotlib 2.1.0:
    # @ref https://github.com/matplotlib/matplotlib/issues/9345
    mpl_ver=$(python -c "import matplotlib as mpl; print(mpl.__version__)")
    if [[ ${mpl_ver} == "2.1.0" ]]; then
        no_plotting=1
    fi
fi

cc_bin_flags=
if [[ ${no_plotting} == 1 ]]; then
    cc_bin_flags='--gtest_filter=-TestCallPython.Plot*'
fi

py-error() {
    echo "ERROR: Python client did not exit successfully."
    exit 1
}

pause() {
    # General busy-spinning.
    sleep 0.05
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
    echo -e "\n\n\n[ ${func}: nominal ]"
    do-setup 0 0
    ${func}
    # Sub-case 2: With Error
    echo -e "\n\n\n[ ${func}: with_error ]"
    do-setup 1 0
    ${func}
    # Sub-case 3: With Error + Stop on Error
    echo -e "\n\n\n[ ${func}: with_error + stop_on_error ]"
    do-setup 1 1
    ${func}
}

py-check() {
    # Check the status of the Python executable (either `wait ...` or the
    # executable itself).
    if [[ ${py_fail} -eq 0 ]]; then
        # Should succeed.
        "$@" || py-error
    else
        # Should fail.
        # TODO(eric.cousineau): File / find bug in Bash for this; this behaves
        # differently depending on how this is placed in a function.
        { "$@" && should-fail; } || :
    fi
}

SIGPIPE_STATUS=141

cc-check() {
    if [[ ${py_fail} -eq 0 ]]; then
        "$@" || { echo "C++ binary failed"; exit 1; }
    else
        # If the C++ binary has not finished by the time the Python client
        # exits due to failure, then the C++ binary will fail with SIGPIPE.
        set +e
        "$@"
        status=$?
        set -e
        if [[ ${status} -eq 0 ]]; then
            :
        elif [[ ${status} -eq ${SIGPIPE_STATUS} ]]; then
            echo "C++ binary failed with SIGPIPE; expected behavior, continuing."
        else
            echo "C++ binary failed"
            exit ${status}
        fi
    fi
}

do-setup() {
    py_fail=${1}
    py_stop_on_error=${2}

    cc_flags="--file=${filename} --done_file=${done_file}"
    if [[ ${py_fail} -eq 1 ]]; then
        cc_flags="${cc_flags} --with_error"
    fi
    py_flags="--file=${filename}"
    if [[ ${py_stop_on_error} -eq 1 ]]; then
        py_flags="${py_flags} --stop_on_error"
    fi

    rm -f ${filename}
    if [[ ${use_fifo} -eq 1 ]]; then
        mkfifo ${filename}
    fi
    echo 0 > ${done_file}
}

do-kill-after() {
    pid=${1}
    done_count_max=${2}
    # Ensure that we wait until the client is fully done with both
    # executions.
    done_count=0
    while [[ ${done_count} -lt ${done_count_max} ]]; do
        done_count=$(cat ${done_file})
        pause
    done
    # Kill the client with Ctrl+C.
    # TODO(eric.cousineau): In script form, this generally works well (only
    # one interrupt needed); however, interactively we need a few more.
    while ps -p ${pid} > /dev/null; do
        kill -INT ${pid} || :
        pause
    done
}

# Execute tests using FIFO.
use_fifo=1

no_threading-no_wait() {
    # Start Python binary in the background.
    ${py_client_cli} --no_threading --no_wait ${py_flags} &
    pid=$!
    # Execute C++.
    cc-check ${cc_bin} ${cc_bin_flags} ${cc_flags}
    # When this is done, Python client should exit.
    py-check wait ${pid}
}
sub-tests no_threading-no_wait

threading-no_wait() {
    ${py_client_cli} --no_wait ${py_flags} &
    pid=$!
    cc-check ${cc_bin} ${cc_bin_flags} ${cc_flags}
    py-check wait ${pid}
}
sub-tests threading-no_wait

threading-wait() {
    ${py_client_cli} ${py_flags} &
    pid=$!
    cc-check ${cc_bin} ${cc_bin_flags} ${cc_flags}
    if [[ ${py_stop_on_error} -ne 1 ]]; then
        # If the client will not halt execution based on an error, execute C++
        # client once more.
        cc-check ${cc_bin} ${cc_bin_flags} ${cc_flags}
        do-kill-after ${pid} 2
    fi
    py-check wait ${pid}
}
# TODO(eric.cousineau): Re-enable this on Mac once the root cause is identified
# for failure on CI machines.
if [[ -z ${is_mac} ]]; then
    sub-tests threading-wait
else
    echo "SKIPPING: sub-tests threading-wait"
fi


# Execute tests without FIFO.
use_fifo=0

no_fifo-no_threading-no_wait() {
    # Execute C++ first.
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    # Start Python binary to consume generated file.
    py-check ${py_client_cli} --no_threading --no_wait ${py_flags}
}
sub-tests no_fifo-no_threading-no_wait

no_fifo-threading-no_wait() {
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    py-check ${py_client_cli} --no_wait ${py_flags}
}
sub-tests no_fifo-threading-no_wait

no_fifo-no_threading-wait() {
    # Execute C++ first.
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    # Start Python binary to consume generated file.
    ${py_client_cli} --no_threading ${py_flags} &
    pid=$!
    if [[ ${py_stop_on_error} -ne 1 ]]; then
        do-kill-after ${pid} 1
    fi
    py-check wait ${pid}
}
sub-tests no_fifo-no_threading-wait

no_fifo-threading-wait() {
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    ${py_client_cli} ${py_flags} &
    pid=$!
    if [[ ${py_stop_on_error} -ne 1 ]]; then
        do-kill-after ${pid} 1
    fi
    py-check wait ${pid}
}
sub-tests no_fifo-threading-wait
