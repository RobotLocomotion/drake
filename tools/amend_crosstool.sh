#!/bin/bash
# This script takes one argument, a path to a crosstool file. It adds some additional
# compiler flags to the toolchain stanzas in that crosstool file, and writes the output
# to CROSSTOOL in the working directory.

# This is a terrible hack. We use it to enable different compiler warnings and errors
# on different platforms, while still taking advantage of Bazel auto-configuration.
# It also may not be the right thing to do, since it forces our flag choices onto all
# the externals.
# TODO(david-german-tri): Improve this if possible.

# Extract all the compilers mentioned in the input file, and grep through their
# --version output to see if gcc, clang, or both are mentioned. Ignore garbage
# "compilers" like /bin/false, and prefix cc_wrapper.sh with "./" so that it executes.
compiler_version_commands=$(mktemp)
grep -o 'tool_path.*gcc.*' "$1" \
    | sed 's/.*path: "\(.*\)" }/\1 --version/' \
    | sed 's/cc_wrapper\.sh/.\/cc_wrapper.sh/' \
    | grep -v '/bin/false' \
    > $compiler_version_commands
source $compiler_version_commands | grep gcc > /dev/null
has_gcc=$?
source $compiler_version_commands | grep clang > /dev/null
has_clang=$?

extra_warnings=$(mktemp)

if [ $has_gcc -eq 0 -a $has_clang -eq 0 ]; then
	echo "CROSSTOOL mentions both clang and gcc.  Aborting."
	exit 1
elif [ $has_gcc -eq 0 ]; then
    # Copy the input file into CROSSTOOL.
    cat "$1" > CROSSTOOL
	# Assemble additional warnings for gcc.
	echo '  # Additional GCC warnings from amend_crosstool.sh' >> $extra_warnings
	echo '  cxx_flag: "-Werror=extra"' >> $extra_warnings
    echo '  cxx_flag: "-Werror=return-local-addr"' >> $extra_warnings
    echo '  cxx_flag: "-Wno-unused-parameter"' >> $extra_warnings
    echo '  cxx_flag: "-Wno-missing-field-initializers"' >> $extra_warnings
	echo '  # End of additional GCC warnings from amend_crosstool.sh' >> $extra_warnings
    # Inject the warnings into the "local" configuration.
    sed -i -e "/^toolchain {/r ${extra_warnings}" CROSSTOOL
	exit 0
elif [ $has_clang -eq 0 ]; then
    # Copy the input file into CROSSTOOL.
    cat "$1" > CROSSTOOL
	# Assemble additional warnings for clang.
	echo '  # Additional CLANG warnings from amend_crosstool.sh' >> $extra_warnings
    echo '  cxx_flag: "-Werror=inconsistent-missing-override"' >> $extra_warnings
    echo '  cxx_flag: "-Werror=sign-compare"' >> $extra_warnings
    echo '  cxx_flag: "-Werror=return-stack-address"' >> $extra_warnings
	echo '  # End of additional CLANG warnings from amend_crosstool.sh' >> $extra_warnings
    # Inject the warnings into the "local" configuration.
    sed -i -e "/^toolchain {/r ${extra_warnings}" CROSSTOOL
	exit 0
else
	echo "CROSSTOOL mentions neither clang nor gcc. Aborting."
	exit 1
fi
