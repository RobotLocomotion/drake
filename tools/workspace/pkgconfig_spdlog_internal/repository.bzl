load("//tools/workspace:pkg_config.bzl", "setup_pkg_config_repository")

def _impl(repo_ctx):
    # N.B. We do not check the return value here for errors. Sometimes this
    # rule is evaluated on systems where pkg-config will fail but where the
    # package will not actually be used. Instead, we'll rely on the deferred
    # reporting inside the generated BUILD file to report the error message
    # later on, if and only if the library is actually used.
    setup_pkg_config_repository(repo_ctx)

pkgconfig_spdlog_internal_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "spdlog"),
        # Offered for backwards compatibility, but ignored.
        "mirrors": attr.string_list_dict(),
        # TODO(jwnimmer-tri) Remove this line when we drop WORKSPACE support.
        "extra_defines": attr.string_list(default = ["HAVE_SPDLOG"]),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
