# About

This is a stub implementation of `bazel_external_data`.
More functionality to follow.

# Structure

This presently performs meta-testing with Bazel to ensure that we achieve the
desired workflows with Bazel. This is all structured such that `bazel test ...`
is valid from Drake, and from each test workspace under  `test/`.

The structure:

*   `BUILD.bazel` - Effectively a no-op; only for exposing files.
*   `external_data.bzl` - Macros for using external data.
*   `expose_all_files.bzl` - Macros to allow for (recursive) consumption of
files from separate packages / workspaces.
*   `test/`
    *   `BUILD.bazel` - Declares tests (unlike other Drake packages), declares
    linting for all of `//tools/external_data/...`.
    *   `workspace_test.bzl`, `workspace_test.sh` - For testing separate
    workspaces, providing a writeable copy of `*.runfiles`.
    *   `external_data_workspace_tests.bzl` - Provides a list of workspaces to
    be tested, repository declarations, convenience wrappings for linting
    files, and macro for `external_data_workspace_test.sh`.
    *   `remove_bazel_symlinks.sh` - Script to remove `bazel-*` symlinks (which
    mess up Bazel's own package scanning).
    *   `external_data_*_test` - Workspaces for testing downstream behavior.
        * `external_data_pkg_test` - Stub for now.
