load("//tools/workspace:pkg_config.bzl", "setup_pkg_config_repository")

# Deprecated for removal on 2026-02-01.

def _impl(repo_ctx):
    # N.B. We do not check the return value here for errors. Sometimes this
    # rule is evaluated on systems where pkg-config will fail but where the
    # package will not actually be used. Instead, we'll rely on the deferred
    # reporting inside the generated BUILD file to report the error message
    # later on, if and only if the library is actually used.
    setup_pkg_config_repository(repo_ctx)

pkgconfig_x11_internal_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "x11"),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
