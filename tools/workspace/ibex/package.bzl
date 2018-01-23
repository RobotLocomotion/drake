# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def ibex_repository(name, modname = "ibex", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
