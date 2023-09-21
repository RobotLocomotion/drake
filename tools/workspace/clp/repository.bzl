load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def clp_repository(
        name,
        licenses = [
            "reciprocal",  # EPL-2.0
        ],
        modname = "clp",
        pkg_config_paths = [],
        homebrew_subdir = "opt/clp/lib/pkgconfig",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        homebrew_subdir = homebrew_subdir,
        extra_deprecation = "The @clp external is deprecated in Drake's WORKSPACE and will be removed on or after 2023-12-01.",  # noqa
        **kwargs
    )
