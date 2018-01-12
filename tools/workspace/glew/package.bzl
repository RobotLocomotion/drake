# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def glew_repository(name, modname = "glew", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
