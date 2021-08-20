# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def jsoncpp_repository(
        name,
        licenses = [
            "notice",  # MIT
            "unencumbered",  # Public-Domain
        ],
        modname = "jsoncpp",
        pkg_config_paths = ["/usr/local/opt/jsoncpp/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        extra_deprecation = "DRAKE DEPRECATED: The @jsoncpp external will be removed from Drake on or after 2021-12-01.",  # noqa
        **kwargs
    )
