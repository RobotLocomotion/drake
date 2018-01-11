# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def dreal_repository(name, modname = "dreal", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
