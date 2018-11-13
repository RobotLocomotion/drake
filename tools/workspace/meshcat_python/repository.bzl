# -*- mode: python -*-
# vi: set ft=python :

"""
Makes the meshcat module from meshcat-python available to be used as a Python
dependency. A meshcat-server console script is also created. On all platforms,
a meshcat-python archive is downloaded from GitHub (https://git.io/fxbL0) or a
specified mirror and unpacked.

Example:
    WORKSPACE:
        load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
        load(
            "@drake//tools/workspace/meshcat_python:repository.bzl",
            "meshcat_python_repository",
        )
        meshcat_python_repository(name = "foo", mirrors = DEFAULT_MIRRORS)

    BUILD:
        py_library(
            name = "foobar",
            deps = ["@foo//:meshcat"],
            srcs = ["bar.py"],
        )

    Command Line:
        $ bazel run @foo//:meshcat-server

Arguments:
    name: A unique name for this rule. The rule must not be named meshcat.
    mirrors: A dictionary of mirrors, see tools/workspace/mirrors.bzl for an
             example.
"""

def _impl(repository_ctx):
    if repository_ctx.name == "meshcat":
        fail("Rule must NOT be named meshcat")

    urls = [
        x.format(
            repository = "rdeits/meshcat-python",
            commit = "d0c8b6a9d1d750495ef9513254761cc14773cf99",
        )
        for x in repository_ctx.attr.mirrors.get("github")
    ]
    repository_ctx.download_and_extract(
        urls,
        sha256 = "99cdea957adf585b33c83cb284b6df16f953b0422ac5a035f4e0f50cf9105121",  # noqa
        stripPrefix = "meshcat-python-d0c8b6a9d1d750495ef9513254761cc14773cf99",  # noqa
    )

    repository_ctx.symlink(
        Label("@drake//tools/workspace/meshcat_python:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    # src/meshcat/viewer is a git submodule and so not included in the
    # meshcat-python archive (https://git.io/fpUnO). Therefore, we download it
    # separately and symlink the necessary files instead.

    repository_ctx.symlink(
        Label("@meshcat//:dist/index.html"),
        "src/meshcat/viewer/dist/index.html",
    )

    repository_ctx.symlink(
        Label("@meshcat//:dist/main.min.js"),
        "src/meshcat/viewer/dist/main.min.js",
    )

meshcat_python_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
