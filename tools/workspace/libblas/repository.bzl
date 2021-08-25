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
    # TLDR: Use @blas instead of this repository (@libblas) in the deps of your
    # target.
    #
    # On Ubuntu, we will use pkg-config to find libblas.so (the vendor of which
    # is typically chosen by the Ubuntu alternatives system). On macOS, no
    # targets should depend on @libblas. However, on both macOS and Ubuntu,
    # targets should normally depend on the alias @blas instead of @libblas (or
    # @openblas) to install an operating-system-specific BLAS implementation.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu or os_result.is_manylinux:
        error = setup_pkg_config_repository(repo_ctx).error
        if error != None:
            fail(error)
    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/libblas:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )

libblas_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "blas"),
        "licenses": attr.string_list(default = ["notice"]),  # BSD-3-Clause
    },
    local = True,
    configure = True,
    implementation = _impl,
)
