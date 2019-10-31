# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def gflags_repository(
        name,
        licenses = ["notice"],  # BSD-3-Clause
        modname = "gflags",
        pkg_config_paths = ["/usr/local/opt/gflags/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        extra_deps = ["@drake//tools/workspace/gflags:pthread_iff_linux"],
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
