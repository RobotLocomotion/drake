# -*- python -*-

# Include release options:
# https://docs.bazel.build/versions/master/user-manual.html#bazel-releng
ARGS_DEFAULT = [
    "bazel",
    "--bazelrc=/dev/null",
    "--batch",
    "test",
    "//...",
]

def workspace_test(
        name,
        size = None,
        timeout = None,
        args = ARGS_DEFAULT,
        data = []):
    """
    Copies all contents under `*.runfiles/${workspace}/**` to a temporary
    directory, then evalutates `args` in `bash` in the new temporary runfiles
    workspace directory.

    @param args
        Arguments for `bash` to execute. By default, will run
        `bazel test //...` with release flavoring.
    @param data
        Data required for the workspace test.
    """
    if size == None:
        size = "small"
    native.sh_test(
        name = name,
        size = size,
        timeout = timeout,
        srcs = ["@drake//tools/external_data/test:workspace_test.sh"],
        args = args,
        data = data,
    )
