# -*- python -*-

# Include release options:
# https://docs.bazel.build/versions/master/user-manual.html#bazel-releng
# N.B. We do not need quotes because these arguments will be passed to
# `eval "$@"`.
CMD_DEFAULT = "bazel --bazelrc=/dev/null --batch test //..."

def workspace_test(
        name,
        args = [CMD_DEFAULT],
        data = []):
    """Copies all contents under `*.runfiles/${workspace}/**` to a temporary
    directory, then evalutates `args` in `bash` in the new temporary runfiles
    workspace directory.
    """
    native.sh_test(
        name = name,
        srcs = ["@drake//tools/external_data/test:workspace_test.sh"],
        args = args,
        data = data,
    )
