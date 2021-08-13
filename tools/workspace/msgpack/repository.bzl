# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def msgpack_repository(
        name,
        licenses = ["notice"],  # Boost-1.0
        modname = "msgpack",
        pkg_config_paths = ["/usr/local/opt/msgpack/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
