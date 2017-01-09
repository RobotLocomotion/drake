#!/bin/bash
# This script takes one argument, a path to a crosstool file. It adds some additional
# compiler flags to the toolchain stanzas in that crosstool file, and writes the output
# to CROSSTOOL in the working directory.

# This is a terrible hack. We use it to enable different compiler warnings and errors
# on different platforms, while still taking advantage of Bazel auto-configuration.
# It also may not be the right thing to do, since it forces our flag choices onto all
# the externals.
#
# See https://github.com/bazelbuild/bazel/issues/2358.
#
# TODO(david-german-tri): Improve this if possible.

# Extract all the compilers mentioned in the input file, and grep through their
# --version output to see if gcc, clang, or both are mentioned. Ignore garbage
# "compilers" like /bin/false, and prefix cc_wrapper.sh with "./" so that it
# executes.
compiler_version_commands=$(mktemp)
grep -o 'tool_path.*gcc.*' "$1" |
    sed 's/.*path: "\(.*\)" }/\1 --version/' |
    sed 's/cc_wrapper\.sh/.\/cc_wrapper.sh/' |
    grep -v '/bin/false' > $compiler_version_commands

compiler_versions=$(mktemp)
source $compiler_version_commands > $compiler_versions
if [ $? -ne 0 ]; then
    echo "Obtaining compiler versions failed. Aborting."
    exit 1
fi
grep gcc $compiler_versions > /dev/null
has_gcc=$?
grep clang $compiler_versions > /dev/null
has_clang=$?

extra_warnings=$(mktemp)

# A function of two arguments, both of which are files. The first file has
# one compiler flag per line. Reads each line, wraps it in a CROSSTOOL protobuf
# cxx_flag field, and appends the result onto the second file.
read_options_into() {
    sed -e 's/^/  cxx_flag: "/' "$1" | sed -e 's/$/"/' >> "$2"
}

if [ $has_gcc -eq 0 -a $has_clang -eq 0 ]; then
    echo "CROSSTOOL mentions both clang and gcc.  Aborting."
    exit 1
elif [ $has_gcc -eq 0 ]; then
    # Copy the input file into CROSSTOOL.
    cat "$1" > CROSSTOOL
    # Assemble additional warnings for gcc.
    echo '  # GCC warnings from amend_crosstool.sh' >> $extra_warnings
    read_options_into "gcc_options.txt" $extra_warnings
    echo '  # End of GCC warnings from amend_crosstool.sh' >> $extra_warnings
elif [ $has_clang -eq 0 ]; then
    # Copy the input file into CROSSTOOL.
    cat "$1" > CROSSTOOL
    # Assemble additional warnings for clang.
    echo '  # CLANG warnings from amend_crosstool.sh' >> $extra_warnings
    read_options_into "clang_options.txt" $extra_warnings
    echo '  # End of CLANG warnings from amend_crosstool.sh' >> $extra_warnings
else
    echo "CROSSTOOL mentions neither clang nor gcc. Aborting."
    exit 1
fi

# Inject the warnings into the every toolchain in the CROSSTOOL.
sed -i -e "/^toolchain {/r ${extra_warnings}" CROSSTOOL
exit 0
