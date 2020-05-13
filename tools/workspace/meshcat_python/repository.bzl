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
        load("//tools/skylark:py.bzl", "py_library")
        py_library(
            name = "foobar",
            deps = ["@foo//:meshcat"],
            srcs = ["bar.py"],
        )

    Command Line:
        $ bazel run @foo//:meshcat-server

Args:
    name: A unique name for this rule. The rule must not be named meshcat.
    mirrors: A dictionary of mirrors, see tools/workspace/mirrors.bzl for an
        example.
"""

load("@drake//tools/workspace:github.bzl", "github_download_and_extract")

def _impl(repository_ctx):
    if repository_ctx.name == "meshcat":
        fail("Rule must NOT be named meshcat")

    github_download_and_extract(
        repository_ctx,
        "rdeits/meshcat-python",
        "ceb8bd68e3f4711b30001876efc5b49530038866",
        repository_ctx.attr.mirrors,
        sha256 = "adcae8290f6e3bcba349ce97a9543150da03606d7fed352a7d26c5e0413dce46",  # noqa
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
