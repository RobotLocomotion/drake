# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def clp_repository(
        name,
        licenses = [
            "reciprocal",  # EPL-2.0
        ],
        modname = "clp",
        pkg_config_paths = [
            "/usr/local/opt/clp/lib/pkgconfig",
        ],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
