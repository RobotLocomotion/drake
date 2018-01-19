# -*- python -*-

load(
    "@com_google_protobuf//:protobuf.bzl",
    "proto_gen",
    "py_proto_library",
)
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_library",
)
load("//tools/skylark:6996.bzl", "adjust_labels_for_drake_hoist")

def drake_cc_proto_library(
        name,
        srcs = [],
        deps = [],
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    deps = adjust_labels_for_drake_hoist(deps)
    pb_hdrs = [x[:-len(".proto")] + ".pb.h" for x in srcs]
    pb_srcs = [x[:-len(".proto")] + ".pb.cc" for x in srcs]
    # Generate the h and cc file.
    proto_gen(
        name = name + "_genproto",
        srcs = srcs,
        deps = [s + "_genproto" for s in deps],
        protoc = "@com_google_protobuf//:protoc",
        gen_cc = 1,
        outs = pb_srcs + pb_hdrs,
        visibility = ["//visibility:public"],
    )
    # Apply ubsan fixups.
    pb_ubsan_fixups = [x[:-len(".proto")] + "_ubsan_fixup.pb.cc" for x in srcs]
    tool = "//tools/skylark:drake_proto_ubsan_fix"
    native.genrule(
        name = name + "_ubsan_fixup",
        srcs = pb_srcs,
        outs = pb_ubsan_fixups,
        tools = [tool],
        cmd = "$(location {tool}) '$(SRCS)' '$(OUTS)'".format(tool = tool),
    )
    # Compile the cc files.
    drake_cc_library(
        name = name,
        srcs = pb_ubsan_fixups,
        hdrs = pb_hdrs,
        tags = tags + ["nolint"],
        deps = [
            "@com_google_protobuf//:protobuf",
            "@drake//common/proto:protobuf_ubsan_fixup",
        ],
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
