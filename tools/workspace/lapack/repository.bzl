# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def lapack_repository(name, modname = "lapack", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
