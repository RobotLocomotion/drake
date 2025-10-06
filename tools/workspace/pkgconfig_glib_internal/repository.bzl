load("//tools/workspace:pkg_config.bzl", "setup_pkg_config_repository")

def _impl(repo_ctx):
    # N.B. We do not check the return value here for errors. Sometimes this
    # rule is evaluated on systems where pkg-config will fail but where the
    # package will not actually be used. Instead, we'll rely on the deferred
    # reporting inside the generated BUILD file to report the error message
    # later on, if and only if the library is actually used.
    setup_pkg_config_repository(repo_ctx)

pkgconfig_glib_internal_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "glib-2.0"),
        "extra_build_file_templates": attr.string_keyed_label_dict(
            default = {
                "glib/BUILD.bazel": "//tools/workspace/pkgconfig_glib_internal:glib.BUILD.bazel",  # noqa
            },
        ),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
