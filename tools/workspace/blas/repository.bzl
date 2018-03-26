# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def blas_repository(
        name,
        licenses = ["notice"],  # BSD-3-Clause
        modname = "blas",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        **kwargs)
