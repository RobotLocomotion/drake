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
    # On Ubuntu, we'll use use pkg-config to find libblas.
    # On macOS, no targets should depend on @blas.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        error = setup_pkg_config_repository(repo_ctx).error
        if error != None:
            fail(error)
    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/blas:package-macos.BUILD.bazel"),
            "BUILD.bazel")

blas_repository = repository_rule(
    # TODO(jamiesnape): Pass down licenses to setup_pkg_config_repository.
    # The license for this package should be:
    #    licenses(["notice"])  # BSD-3-Clause
    attrs = {
        "modname": attr.string(default = "blas"),
    },
    local = True,
    implementation = _impl,
)
