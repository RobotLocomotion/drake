# About

This is a stub implementation of `bazel_external_data`.
More functionality to follow.

# Structure

This presently performs meta-testing with Bazel to ensure that we achieve the
desired workflows with Bazel. This is all structured such that `bazel test ...`
is valid from Drake, and from `./workspace/`. (It can work in the downstream
test packages too, but Drake features must be stubbed.)

The structure:

*   `workspace/` - The local workspace for `bazel_external_data_pkg`.
    *   This is structured such that this can be devloped relatively independently of Drake, for the purpose of testing downstream behavior with minimal instrumentation.
    *   `tools/macros.bzl` - Macros to be consumed by downstream workspaces.
    *   `test/workspaces/` - Workspaces to test usage of this repository.
        *   `bazel_pkg_test/` - Basic test. (Presently stubbed.)
*   `./`
    *   `external_data.bzl` - Macros from `bazel_external_data_pkg`, configured
    for Drake.
    *   `BUILD.bazel` - Defines `:workspace_test`. See `test/workspace_test.sh` for description.
    *   `test/`
        *   `workspace_test.sh` - Meta-test which will run tests from `bazel_external_data_pkg` as a standalone workspace.
