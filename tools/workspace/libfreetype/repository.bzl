# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def libfreetype_repository(
        name,
        licenses = ["notice"],  # FreeType License
        modname = "freetype2",
        pkg_config_paths = [],
        homebrew_subdir = "opt/freetype/lib/pkgconfig",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
