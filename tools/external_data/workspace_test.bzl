# -*- python -*-

# Needs quotes, because `sh_test(args = [...])` just concatenates the arguments.
_CMD_DEFAULT = "'bazel test //...'"

def workspace_test(
        name,
        workspace,
        cmd = _CMD_DEFAULT,
        data = []):
    """Provides a unittest access to a given workspace
    contained in the current project.

    @param workspace
        Workspace directory relative to this package.
    @param cmd
        Command to run. By default is `bazel test //...`.
    @param data
        Additional data (e.g. other workspaces).
    """

    args = [cmd, "$(location {})".format(workspace)]
    for datum in data:
        args.append("$(locations {})".format(datum))
    native.sh_test(
        name = name,
        # TODO(eric.cousineau): Is it possible get the package of the *current*
        # macro file (rather than the current BUILD file)?
        srcs = ["@drake//tools/external_data:workspace_writeable_test.sh"],
        args = args,
        data = [workspace] + data,
    )

def glob_lint_sources(base_dir):
    """Globs the relevant lint sources for use with `add_lint_tests` in a
    directory (which may be a workspace or set of workspaces). """
    return struct(
        bazel_srcs = native.glob([
            base_dir + "/**/BUILD.bazel",
            base_dir + "/**/WORKSPACE",
            base_dir + "/**/*.bzl",
        ]),
        python_srcs = native.glob([
            base_dir + "/**/*.py",
        ]),
        cc_srcs = native.glob([
            base_dir + "/**/*.h",
            base_dir + "/**/*.cc",
        ]),
    )
