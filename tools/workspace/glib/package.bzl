# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def glib_repository(name, modname = "glib-2.0", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
