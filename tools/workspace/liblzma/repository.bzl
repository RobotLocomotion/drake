load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def liblzma_repository(
        name,
        licenses = ["unencumbered"],  # Public-Domain
        modname = "liblzma",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        extra_deprecation = "The @liblzma external is deprecated in Drake's WORKSPACE and will be removed on or after 2024-01-01.",  # noqa
        **kwargs
    )
