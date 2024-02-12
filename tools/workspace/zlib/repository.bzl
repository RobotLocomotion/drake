"""
Makes a system-installed zlib available to be used as a C/C++ dependency.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/zlib:repository.bzl", "zlib_repository")
        zlib_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:zlib"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

def _impl(repository_ctx):
    repository_ctx.symlink(
        Label("@drake//tools/workspace/zlib:package.BUILD.bazel"),
        "BUILD.bazel",
    )

zlib_repository = repository_rule(
    implementation = _impl,
)
