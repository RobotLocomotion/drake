load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def clp_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["reciprocal"],  # EPL-2.0
        modname = "clp",
        extra_deprecation = "The @clp external is deprecated in Drake's WORKSPACE and will be removed on or after 2023-12-01.",  # noqa
    )
