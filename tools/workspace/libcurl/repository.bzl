# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def libcurl_repository(
        name,
        licenses = ["notice"],  # curl
        modname = "libcurl",
        pkg_config_paths = [],
        homebrew_subdir = "opt/curl/lib/pkgconfig",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        extra_deprecation = "DRAKE DEPRECATED: The @libcurl external is being removed from Drake on or after 2022-12-01.  Downstream projects should add it to their own WORKSPACE if needed.",  # noqa
        **kwargs
    )
