# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def gthread_repository(
        name,
        licenses = ["restricted"],  # LGPL-2.0+
        modname = "gthread-2.0",
        pkg_config_paths = ["/usr/local/opt/glib/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs)
