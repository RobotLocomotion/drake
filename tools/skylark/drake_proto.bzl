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
        tags = tags + ["nolint"],
        **kwargs)

def drake_py_proto_library(
        name,
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    py_proto_library(
        name = name,
        tags = tags + ["nolint"],
        **kwargs)
