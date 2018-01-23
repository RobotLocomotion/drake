# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def blas_repository(name, modname = "blas", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
