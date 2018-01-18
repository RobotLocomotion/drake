# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def gthread_repository(name, modname = "gthread-2.0", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
