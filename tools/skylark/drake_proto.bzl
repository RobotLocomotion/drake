# -*- python -*-

load(
    "@protobuf//:protobuf.bzl",
    "cc_proto_library",
    "py_proto_library",
)

def drake_cc_proto_library(
        name,
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    cc_proto_library(
        name = name,
        protoc = "@drake//tools/skylark:drake_protoc",
        tags = tags + ["nolint"],
        default_runtime = "@systemprotobuf",
        **kwargs)

def drake_py_proto_library(
        name,
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    py_proto_library(
        name = name,
        protoc = "@drake//tools/skylark:drake_protoc",
        default_runtime = None,  # use the system default
        tags = tags + ["nolint"],
        **kwargs)
