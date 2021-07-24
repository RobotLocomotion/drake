def _recursive_filegroup_impl(ctx):
    files = depset([], transitive = [
        d.data_runfiles.files
        for d in ctx.attr.data
    ])
    return [DefaultInfo(
        files = files,
        data_runfiles = ctx.runfiles(
            files = files.to_list(),
        ),
    )]

"""
Provides all files (including `data` dependencies) at one level such that they
are expandable via `$(locations ...)`.

@param data
    Upstream data targets. This will consume both the `srcs` and `data`
    portions of an existing `filegroup`.
"""

recursive_filegroup = rule(
    attrs = {
        "data": attr.label_list(
            allow_files = True,
            mandatory = True,
        ),
    },
    implementation = _recursive_filegroup_impl,
)

# Patterns to be exposed.
_patterns_map = dict(
    all_files = [
        "*",
    ],
    # To minimize the number of dependencies need to consume `external_data`,
    # this list is copied (not imported) from `//tools/lint:bazel_lint.bzl`.
    bazel_lint_files = [
        "*.bzl",
        "*.BUILD",
        "*.BUILD.bazel",
        "BUILD",
        "BUILD.bazel",
        "WORKSPACE",
    ],
    python_lint_files = [
        "*.py",
    ],
)

def expose_all_files(
        sub_packages = [],
        sub_dirs = [],
        visibility = ["//visibility:public"]):
    """
    Declares files to be consumed externally (for Bazel workspace tests,
    linting, etc).
    Creates rules "${type}_files" and "${type}_files_recursive", where `type`
    will be all of {"all", "bazel_lint", "python_lint"}.

    @param sub_packages
        Child packages, only the first level.
    @param sub_dirs
        Any directories that are not packages.
    """
    print("The expose_all_files.bzl macro is deprecated and will be removed from Drake on or after 2021-11-01.")  # noqa

    # @note It'd be nice if this could respect *ignore files, but meh.
    # Also, it'd be **super** nice if Bazel did not let `**` globs leak into
    # other packages and then error out.
    package_name = native.package_name()
    if package_name:
        package_prefix = "//" + package_name + "/"
    else:
        package_prefix = "//"  # Root case.
    for name, patterns in _patterns_map.items():
        srcs = native.glob(patterns)
        for sub_dir in sub_dirs:
            srcs += native.glob([
                sub_dir + "/" + pattern
                for pattern in patterns
            ])
        native.filegroup(
            name = name,
            srcs = srcs,
            # Trying to use `data = deps` here only exposes the files in
            # runfiles, but not for expansion via `$(locations...)`.
            visibility = visibility,
        )

        # Expose all files recursively (from one level).
        deps = [
            package_prefix + sub_package + ":" + name + "_recursive"
            for sub_package in sub_packages
        ]
        recursive_filegroup(
            name = name + "_recursive",
            data = [name] + deps,
            visibility = visibility,
        )
