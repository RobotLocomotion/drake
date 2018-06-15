# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def liblz4_repository(
        name,
        licenses = ["notice"],  # BSD-2-Clause
        modname = "liblz4",
        pkg_config_paths = ["/usr/local/opt/lz4/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
