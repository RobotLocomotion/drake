load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def x11_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # X11/MIT
        modname = "x11",
        defer_error_os_names = ["mac os x"],
    )
