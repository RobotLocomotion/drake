load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def libtiff_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # Libtiff
        modname = "libtiff-4",
        extra_deprecation = "The @libtiff external is deprecated in Drake's WORKSPACE and will be removed on or after 2024-02-01.",  # noqa
    )
