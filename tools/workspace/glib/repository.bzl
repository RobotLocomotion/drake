# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def glib_repository(
        name,
        modname = "glib-2.0",
        pkg_config_paths = ["/usr/local/opt/glib/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs)
