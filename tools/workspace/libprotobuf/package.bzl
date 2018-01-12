# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

# Load in the paths and flags to the system version of the protobuf runtime;
# in contrast, the Bazel build rules are loaded as @com_google_protobuf.

def libprotobuf_repository(name, modname = "protobuf", **kwargs):
    pkg_config_repository(name = name, modname = modname, **kwargs)
