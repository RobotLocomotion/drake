# -*- python -*-

load(
    "@com_google_protobuf//:protobuf.bzl",
    "proto_gen",
    "py_proto_library",
)
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_installed_headers",
    "installed_headers_for_drake_deps",
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
    pb_ubsan_fixups = [x[:-len(".proto")] + "_ubsan_fixup.pb.cc" for x in srcs]
    native.genrule(
        name = name + "_ubsan_fixup",
        srcs = pb_srcs,
        outs = pb_ubsan_fixups,
        cmd = "for src in $(SRCS) ; do awk '/#include <google\/protobuf\/generated_message_reflection.h>/{print;print \"#include \\\"protobuf-ubsan-fixup.h\\\"\";next}1' $$src > $${src%??????}_ubsan_fixup.pb.cc ; done",  # noqa
    )
    # Compile the cc file using standard include paths.
    native.cc_library(
        name = name + "_genproto_compile",
        srcs = pb_ubsan_fixups,
        hdrs = pb_hdrs,
        tags = tags + ["nolint"],
        deps = [
            "@com_google_protobuf//:protobuf",
            "@com_google_protobuf//:protobuf_fixup_ubsan",
        ],
        **kwargs)
    # Provide a library with drake-modified include paths, depending on the
    # already-compiled object code.  (We can't compile the .cc file using the
    # drake-modified include paths.)
    if native.package_name().startswith("drake"):
        strip_include_prefix = None
        include_prefix = None
    else:
        # Require include paths like "drake/foo/bar.h", not "foo/bar.h".
        strip_include_prefix = "/"
        include_prefix = "drake"
    native.cc_library(
        name = name,
        hdrs = pb_hdrs,
        tags = tags + ["nolint"],
        strip_include_prefix = strip_include_prefix,
        include_prefix = include_prefix,
        deps = [name + "_genproto_compile"],
        **kwargs)
    # Install the header file.
    drake_installed_headers(
        name = name + ".installed_headers",
        hdrs = pb_hdrs,
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
        tags = tags + ["nolint"],
        **kwargs)
