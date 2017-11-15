# -*- python -*-

load(
    "@protobuf//:protobuf.bzl",
    "cc_proto_library",
    "py_proto_library",
)
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_installed_headers",
    "installed_headers_for_drake_deps",
)

def drake_cc_proto_library(
        name,
        srcs = [],
        deps = [],
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    cc_proto_library(
        name = name,
        srcs = srcs,
        protoc = "@drake//tools/skylark:drake_protoc",
        default_runtime = "@systemprotobuf",
        tags = tags + ["nolint"],
        **kwargs)
    drake_installed_headers(
        name = name + ".installed_headers",
        hdrs = [s[:-len(".proto")] + ".pb.h" for s in srcs],
        deps = installed_headers_for_drake_deps(deps),
        tags = ["nolint"],
    )

def drake_py_proto_library(
        name,
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    py_proto_library(
        name = name,
        protoc = "@drake//tools/skylark:drake_protoc",
        default_runtime = None,  # Use the system default.
        tags = tags + ["nolint"],
        **kwargs)
