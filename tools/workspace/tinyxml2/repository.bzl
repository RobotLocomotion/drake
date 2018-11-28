# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def tinyxml2_repository(
        name,
        licenses = ["notice"],  # Zlib
        modname = "tinyxml2",
        pkg_config_paths = ["/usr/local/opt/tinyxml2/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
