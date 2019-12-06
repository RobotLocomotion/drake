# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

# As of DRAKE_DEPRECATED 2020-02-01 this dependency will become unused by Drake
# and therefore will be removed.

# Load in the paths and flags to the system version of the protobuf runtime.

def libprotobuf_repository(
        name,
        licenses = ["notice"],  # BSD-3-Clause
        modname = "protobuf",
        pkg_config_paths = ["/usr/local/opt/protobuf/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
