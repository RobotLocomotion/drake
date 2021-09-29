# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def liblzma_repository(
        name,
        licenses = ["unencumbered"],  # Public-Domain
        modname = "liblzma",
        pkg_config_paths = ["/usr/local/opt/xz/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
