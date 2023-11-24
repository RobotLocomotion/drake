load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def liblz4_repository(
        name,
        licenses = ["notice"],  # BSD-2-Clause
        modname = "liblz4",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        extra_deprecation = "The @liblz4 external is deprecated in Drake's WORKSPACE and will be removed on or after 2024-01-01.",  # noqa
        **kwargs
    )
