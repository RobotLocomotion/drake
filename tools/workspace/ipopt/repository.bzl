# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def ipopt_repository(name, modname = "ipopt", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
