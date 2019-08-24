# -*- mode: cmake -*-
# vi: set ft=cmake :

list(APPEND CTEST_CUSTOM_COVERAGE_EXCLUDE
  ".*/test/.*"
  ".*/third_party/.*"
)

string(ASCII 27 ESC)

# Note that due to limitations in the CMake language there may only be one
# element in each list containing mismatched opening square brackets
# (i.e., [ without matching ]) and that element must be the last element of the
# list.

# "DEBUG" emitted by Bazel may be colored yellow (CSI 33m), "WARNING" emitted
# by Bazel may be colored magenta (CSI 35m), and "warning" emitted by Clang may
# be colored magenta (CSI 35m) and bolded (CSI 1m).
list(APPEND CTEST_CUSTOM_ERROR_EXCEPTION
  "^DEBUG: "
  "^WARNING: "
  ": warning: "
  ":[0-9]+: Failure$"
  "(^${ESC}\\[33mDEBUG|^${ESC}\\[35mWARNING|: ${ESC}\\[0m${ESC}\\[0\;1\;35mwarning): ${ESC}\\[0m"
)

# "ERROR" emitted by Bazel may be colored red (CSI 31m) and bolded (CSI 1m).
list(APPEND CTEST_CUSTOM_ERROR_MATCH
  "^ERROR: "
  "^${ESC}\\[31m${ESC}\\[1mERROR: ${ESC}\\[0m"
)

# Ignore various Mac CROSSTOOL-related warnings.
list(APPEND CTEST_CUSTOM_WARNING_EXCEPTION
  "ranlib: file: .* has no symbols"
  "ranlib: warning for library: .* the table of contents is empty \\(no object file members in the library define global symbols\\)"
  "warning: argument unused during compilation: '-pie' \\[-Wunused-command-line-argument\\]"
  "warning: '_FORTIFY_SOURCE' macro redefined \\[-Wmacro-redefined\\]"
)

# "WARNING" emitted by Bazel may be colored magenta (CSI 35m) and "warning"
# emitted by Clang may be colored magenta (CSI 35m) and bolded (CSI 1m).
list(APPEND CTEST_CUSTOM_WARNING_MATCH
  "^WARNING: "
  ": warning: "
  "(^${ESC}\\[35mWARNING|: ${ESC}\\[0m${ESC}\\[0\;1\;35mwarning): ${ESC}\\[0m"
)

set(CTEST_CUSTOM_MAXIMUM_NUMBER_OF_ERRORS 100)
set(CTEST_CUSTOM_MAXIMUM_NUMBER_OF_WARNINGS 100)
