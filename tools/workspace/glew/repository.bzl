# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def glew_repository(
        name,
        modname = "glew",
        pkg_config_paths = ["/usr/local/opt/glew/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs)
