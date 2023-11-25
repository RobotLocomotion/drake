load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def glew_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # BSD-3-Clause AND MIT
        modname = "glew",
        extra_deprecation = "The @glew external is deprecated in Drake's WORKSPACE and will be removed on or after 2024-01-01.",  # noqa
    )
