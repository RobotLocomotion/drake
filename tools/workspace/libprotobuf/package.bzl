# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def libprotobuf_repository(name, modname = "protobuf", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
