load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def glx_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # SGI-B-2.0
        modname = "glx",
        extra_deps = [
            "@opengl",
            "@x11",
        ],
        defer_error_os_names = ["mac os x"],
    )
