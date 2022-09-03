# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def tinyxml2_repository(
        name,
        licenses = ["notice"],  # Zlib
        modname = "tinyxml2",
        pkg_config_paths = [],
        homebrew_subdir = "opt/tinyxml2/lib/pkgconfig",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        extra_deprecation = "DRAKE DEPRECATED: The @tinyxml2 alias for the operating system's library is no longer used by Drake. If you still use it, you may copy the @tinyxml2 repository rule into your own project. This target will be removed from Drake on or after 2023-01-01.",  # noqa
        **kwargs
    )
