# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def opengl_repository(
        name,
        # TODO(jeremy.nimmer): Figure out the correct license.
        licenses = ["notice"],
        modname = "gl",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        **kwargs
    )
