# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def liblz4_repository(name, modname = "liblz4", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
