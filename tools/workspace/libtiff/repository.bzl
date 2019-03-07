# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def libtiff_repository(
        name,
        licenses = ["notice"],  # Libtiff
        modname = "libtiff-4",
        pkg_config_paths = ["/usr/local/opt/libtiff/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
