#!/bin/bash

source_file="$1"
patch_file="$2"
output_file="$3"

# Apply the patch, buffering its output.
output=$(
    (
        set -euo pipefail
        cp "${source_file}" "${output_file}"
        patch -p0 "${output_file}" < "${patch_file}"
    ) 2>&1
)
status=$?

# If we failed, then show the buffered output and extra details.
if [[ $status -ne 0 ]]; then
    rm -f "${output_file}"
    echo "ERROR: Could not apply ${patch_file}:" 1>&2
    echo "${output}" 1>&2
    echo "INFO: ${output_file}.rej" 1>&2
    cat "${output_file}.rej" 1>&2
    exit 1
fi

# N.B. Don't show any output if we succeeded.
