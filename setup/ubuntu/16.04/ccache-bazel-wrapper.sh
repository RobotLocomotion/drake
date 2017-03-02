#!/bin/bash

# NOTE: Users and developers are not expected to run this script by hand.

# This script is part of the Drake Developer setup instructions:
# http://drake.mit.edu/developers.html

# This program is meant to be installed as /usr/lib/ccache/bazel, so Bazel and
# ccache play nice.  See https://github.com/RobotLocomotion/drake/issues/4464.

# When a user has /usr/lib/ccache on their $PATH before /usr/bin, the Bazel
# cc_toolchain will try to use, e.g., the /usr/lib/ccache/gcc program found
# there, but that program doesn't work inside the sandbox.  By placing this
# script in the same ccache directory, in exactly the cases that ccache will
# trump GCC, this script will trump Bazel.  Within this script, we remove the
# ccache toolchain from the path, and then shell out to the real Bazel.

# Remove the magic ccache directory from the $PATH.
export PATH=$(
    echo -n "$PATH:" |
        awk -v RS=: -v ORS=: '$0 != "/usr/lib/ccache"' |
        sed 's/:$//')

# Run the real Bazel tool.
exec bazel "$@"
