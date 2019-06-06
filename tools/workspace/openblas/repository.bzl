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
    # On macOS, we'll use use pkg-config to find libopenblas.dylib.
    # On Ubuntu, no targets should depend on @openblas.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_macos:
        error = setup_pkg_config_repository(repo_ctx).error
        if error != None:
            fail(error)
    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/blas:package-ubuntu.BUILD.bazel"),
            "BUILD.bazel",
        )

openblas_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "openblas"),
        "licenses": attr.string_list(default = ["notice"]),  # BSD-3-Clause
        "pkg_config_paths": attr.string_list(
            default = ["/usr/local/opt/openblas/lib/pkgconfig"],
        ),
    },
    local = True,
    implementation = _impl,
)
