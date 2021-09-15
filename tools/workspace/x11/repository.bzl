# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)
load(
    "@drake//tools/workspace:pkg_config.bzl",
    "setup_pkg_config_repository",
)

def _impl(repo_ctx):
    # Only available on Ubuntu. On macOS, no targets should depend on @x11.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu or os_result.is_manylinux:
        error = setup_pkg_config_repository(repo_ctx).error
        if error != None:
            fail(error)
    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/x11:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )

x11_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "x11"),
        "licenses": attr.string_list(default = ["notice"]),  # X11/MIT.
    },
    local = True,
    configure = True,
    implementation = _impl,
)
