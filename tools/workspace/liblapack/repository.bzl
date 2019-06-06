# -*- mode: python -*-

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)
load(
    "@drake//tools/workspace:pkg_config.bzl",
    "setup_pkg_config_repository",
)

def _impl(repo_ctx):
    # On Ubuntu, we'll use use pkg-config to find liblapack.so.
    # On macOS, no targets should depend on @liblapack.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        error = setup_pkg_config_repository(repo_ctx).error
        if error != None:
            fail(error)
    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/lapack:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )

liblapack_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "lapack"),
        "licenses": attr.string_list(default = ["notice"]),  # BSD-3-Clause
    },
    local = True,
    implementation = _impl,
)
