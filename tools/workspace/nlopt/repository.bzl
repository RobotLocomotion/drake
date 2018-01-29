# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def nlopt_repository(name, modname = "nlopt", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
