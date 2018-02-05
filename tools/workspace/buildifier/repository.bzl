# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads a precompiled version of buildifier and makes it available to the
WORKSPACE.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/buildifier:repository.bzl", "buildifier_repository")  # noqa
        buildifier_repository(name = "foo")

    BUILD:
        sh_binary(
            name = "foobar",
            srcs = ["bar.sh"],
            data = ["@foo//:buildifier"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    version = "0.6.0"

    if os_result.is_macos:
        filename = "buildifier.osx"
        sha256 = "631d81d2467a524f9469c923136e41b5b46261939c3a9cf4a84cfbf0a93e7cbb"  # noqa
    elif os_result.is_ubuntu:
        filename = "buildifier"
        sha256 = "04967ce69ed0ff157cc4ba12f8fdeb4db3ed30011183e9b81156ad8dafb1c1fd"  # noqa
    else:
        fail("Operating system is NOT supported", attr = os_result)

    mirrors = [
        "https://github.com/bazelbuild/buildtools/releases/download/%s/%s",
        "https://drake-mirror.csail.mit.edu/github/bazelbuild/buildtools/releases/%s/%s",  # noqa
        "https://s3.amazonaws.com/drake-mirror/github/bazelbuild/buildtools/releases/%s/%s",  # noqa
    ]

    urls = [mirror % (version, filename) for mirror in mirrors]
    output = repository_ctx.path("buildifier")

    repository_ctx.download(urls, output, sha256, executable = True)

    content = """
exports_files(
    ["buildifier"],
)
"""

    repository_ctx.file("BUILD.bazel", content, executable = False)

buildifier_repository = repository_rule(implementation = _impl)
