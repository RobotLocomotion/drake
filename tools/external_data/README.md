# About

This is a stub implementation of incorporating external data.
More functionality to follow.

# For Consumers of `external_data`

The main file of interest is:

*   `external_data.bzl` - Macros for using external data.
*   `cli` - (To be added) Command-line interface.

# For Developers of `external_data`

This presently performs meta-testing with Bazel to ensure that we achieve the
desired workflows with Bazel. This is all structured such that `bazel test ...`
is valid from Drake, and from each test workspace under  `test/`.

Main structure:

*   `BUILD.bazel` - Effectively a no-op; only for exposing files.
*   `expose_all_files.bzl` - Macros to allow for (recursive) consumption of
files from separate packages / workspaces.
*   `bazel_external_data/` - Python code for CLI interface (for both end users
and Bazel `genrule`s).
*   `test/`
    *   `BUILD.bazel` - Declares tests (unlike other Drake packages), declares
    linting for all of `//tools/external_data/...`.
    *   `external_data_workspace_tests.bzl` - Provides a list of workspaces to
    be tested, repository declarations, convenience wrappings for linting
    files, and macro for `external_data_workspace_test.sh`.
    *   `external_data_*_test` - Workspaces for testing downstream behavior.
        * `external_data_pkg_test` - Stub for now.
