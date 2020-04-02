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
    # Only available on Ubuntu. On macOS, no targets should depend on @opengl.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        error = setup_pkg_config_repository(repo_ctx).error
        if error != None:
            fail(error)
    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/opengl:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )

opengl_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "gl"),
        "licenses": attr.string_list(default = ["notice"]),  # SGI-B-2.0.
    },
    local = True,
    configure = True,
    implementation = _impl,
)
