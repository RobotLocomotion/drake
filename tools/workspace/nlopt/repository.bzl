# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def nlopt_repository(
        name,
        licenses = [
            "notice",  # BSD-3-Clause AND MIT
            "restricted",  # LGPL-2.1+
        ],
        modname = "nlopt",
        pkg_config_paths = ["/usr/local/opt/nlopt/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
