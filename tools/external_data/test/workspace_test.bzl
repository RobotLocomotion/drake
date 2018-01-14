# -*- python -*-

# Include release options:
# https://docs.bazel.build/versions/master/user-manual.html#bazel-releng
# Needs quotes, because `sh_test(args = [...])` concatenates the arguments.
CMD_DEFAULT = "'bazel --bazelrc=/dev/null --batch test //...'"

def workspace_test(
        name,
        args = [CMD_DEFAULT],
        data = []):
    """Provides a unittest access to a given workspace
    contained in the current project in a writeable (copied) context.
    """
    native.sh_test(
        name = name,
        # TODO(eric.cousineau): Is it possible get the package of the *current*
        # macro file (rather than the current BUILD file)?
        srcs = [":workspace_test.sh"],
        args = args,
        data = data,
    )
