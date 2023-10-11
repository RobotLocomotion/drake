load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def libtiff_repository(
        name,
        licenses = ["notice"],  # Libtiff
        modname = "libtiff-4",
        pkg_config_paths = [],
        homebrew_subdir = "opt/libtiff/lib/pkgconfig",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        extra_deprecation = "The @libtiff external is deprecated in Drake's WORKSPACE and will be removed on or after 2024-02-01.",  # noqa
        **kwargs
    )
