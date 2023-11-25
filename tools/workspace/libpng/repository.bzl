load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def libpng_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # Libpng
        modname = "libpng",
        extra_deprecation = "The @libpng external is deprecated in Drake's WORKSPACE and will be removed on or after 2024-02-01.",  # noqa
    )
