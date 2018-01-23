# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def tinyxml_repository(name, modname = "tinyxml", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
