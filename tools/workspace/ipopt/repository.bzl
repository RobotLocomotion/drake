# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def ipopt_repository(
        name,
        licenses = [
            "reciprocal",  # CPL-1.0
            "unencumbered",  # Public-Domain
        ],
        modname = "ipopt",
        pkg_config_paths = [
            "/usr/local/opt/ipopt/lib/pkgconfig",
            # TODO(jamiesnape): remove the path to the ipopt@3.11 formula that
            # is below on or after 2021-03-01.
            "/usr/local/opt/ipopt@3.11/lib/pkgconfig",
        ],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
