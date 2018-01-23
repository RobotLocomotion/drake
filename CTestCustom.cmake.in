# -*- mode: cmake -*-
# vi: set ft=cmake :

list(APPEND CTEST_CUSTOM_COVERAGE_EXCLUDE
  ".*/test/.*"
  ".*/third_party/.*"
)

list(APPEND CTEST_CUSTOM_ERROR_EXCEPTION
  "^WARNING: "
  ":[0-9]+: Failure$"
)

list(APPEND CTEST_CUSTOM_ERROR_MATCH
  "^ERROR: "
  "^FAIL: "
  "^TIMEOUT: "
)

# Ignore various Mac CROSSTOOL-related warnings.
list(APPEND CTEST_CUSTOM_WARNING_EXCEPTION
  "ranlib: file: .* has no symbols"
  "ranlib: warning for library: .* the table of contents is empty \\(no object file members in the library define global symbols\\)"
  "warning: argument unused during compilation: '-pie' \\[-Wunused-command-line-argument\\]"
  "warning: '_FORTIFY_SOURCE' macro redefined \\[-Wmacro-redefined\\]"
)

list(APPEND CTEST_CUSTOM_WARNING_MATCH "^WARNING: ")

set(CTEST_CUSTOM_MAXIMUM_NUMBER_OF_ERRORS 100)
set(CTEST_CUSTOM_MAXIMUM_NUMBER_OF_WARNINGS 100)
