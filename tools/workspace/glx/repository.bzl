load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

_DEPRECATION = "DRAKE DEPRECATED: The @glx repository rule is deprecated. If you still need it, you may wish to copy the code for it into your project. The deprecated code will be removed from Drake on or after 2025-05-01."  # noqa

def glx_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # SGI-B-2.0
        modname = "glx",
        extra_deps = [
            "@opengl",
            "@x11",
        ],
        defer_error_os_names = ["mac os x"],
        extra_deprecation = _DEPRECATION,
    )
