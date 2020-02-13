# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def x11_repository(
        name,
        licenses = ["notice"],  # X11/MIT.
        modname = "x11",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        **kwargs
    )
