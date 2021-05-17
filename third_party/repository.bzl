# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)


def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/x11:package-ubuntu.BUILD.bazel"),
            "BUILD.bazel",
        )
    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/x11:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )

x11_repository = repository_rule(
    implementation = _impl,
)