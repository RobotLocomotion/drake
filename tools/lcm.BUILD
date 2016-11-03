# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lcm",
    srcs = [
        "lcm/eventlog.c",
        "lcm/lcm.c",
        "lcm/lcm.h",
        "lcm/lcm_file.c",
        "lcm/lcm_memq.c",
        "lcm/lcm_mpudpm.c",
        "lcm/lcm_tcpq.c",
        "lcm/lcm_udpm.c",
        "lcm/lcmtypes/channel_port_map_update_t.c",
        "lcm/lcmtypes/channel_to_port_t.c",
        "lcm/ringbuffer.c",
        "lcm/udpm_util.c",
    ],
    hdrs = [
        "lcm/dbg.h",
        "lcm/eventlog.h",
        "lcm/ioutils.h",
        "lcm/lcm-cpp-impl.hpp",
        "lcm/lcm-cpp.hpp",
        "lcm/lcm_coretypes.h",
        "lcm/lcm_internal.h",
        "lcm/lcmtypes/channel_port_map_update_t.h",
        "lcm/lcmtypes/channel_to_port_t.h",
        "lcm/ringbuffer.h",
        "lcm/udpm_util.h",
    ],
    includes = ["."],
    copts = ["-Wno-all", "-Wno-deprecated-declarations"],
    deps = ["@gtk//:glib"],
)

cc_binary(
    name = "lcmgen",
    srcs = [
        "lcmgen/emit_c.c",
        "lcmgen/emit_cpp.c",
        "lcmgen/emit_csharp.c",
        "lcmgen/emit_java.c",
        "lcmgen/emit_lua.c",
        "lcmgen/emit_python.c",
        "lcmgen/getopt.c",
        "lcmgen/getopt.h",
        "lcmgen/lcmgen.c",
        "lcmgen/lcmgen.h",
        "lcmgen/main.c",
        "lcmgen/tokenize.c",
        "lcmgen/tokenize.h",
    ],
    includes = ["."],
    copts = [
        "-Wno-all", "-Wno-format-zero-length",
        # TODO(jwnimmer-tri) This hack should be removed when we ugprade
        # to the latest LCM.
        "-include", "unistd.h"],
    deps = [":lcm"],
)
