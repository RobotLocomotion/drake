# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def glew_repository(
        name,
        licenses = ["notice"],  # BSD-3-Clause AND MIT
        modname = "glew",
        pkg_config_paths = ["/usr/local/opt/glew/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
