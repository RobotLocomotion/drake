# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def libpng_repository(name, modname = "libpng", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
