# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def libpng_repository(
        name,
        licenses = ["notice"],  # Libpng
        modname = "libpng",
        pkg_config_paths = ["/usr/local/opt/libpng/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
