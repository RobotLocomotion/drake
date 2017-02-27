#!/bin/bash

# This is a useful stub for sh_test cases whose only interesting content is
# their `args = [...]` as specified by a BUILD file.  For example:
#
#   drake_cc_binary(
#       name = "pendulum_run_swing_up",
#       ...
#   )
#
#   sh_test(
#       name = "pendulum_run_swing_up_test",
#       srcs = ["//tools:exec.sh"],
#       args = [
#           "$(location :pendulum_run_swing_up)",
#           "--target_realtime_rate=0.0",
#       ],
#       data = [":pendulum_run_swing_up"],
#   )
#
# This declares a cc_binary program which has a test-like exitcode (0 for pass,
# non-zero for failure), and then adds it as a test with one extra command-line
# argument so that it runs as quickly as possible.

# The only thing this script needs to do is run the "args" from the sh_test.
exec "$@"
