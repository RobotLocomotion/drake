load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def glib_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["restricted"],  # LGPL-2.0+
        modname = "glib-2.0",
    )
