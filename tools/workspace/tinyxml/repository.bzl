# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def tinyxml_repository(
        name,
        licenses = ["notice"],  # Zlib
        modname = "tinyxml",
        pkg_config_paths = ["/usr/local/opt/tinyxml/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        extra_deprecation = "DRAKE DEPRECATED: The @tinyxml external is being removed from Drake on or after 2021-07-01.  Downstream projects should add it to their own WORKSPACE if needed.",  # noqa
        **kwargs
    )
