# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

# In LCM's upstream build system lcm_export.h is generated by CMake and
# contains preprocessor conditionals for library decoration settings, based on
# what kindsof compilation step is being run.  For our Bazel build of LCM, we
# don't want those values in a header file because we set them directly via our
# defines= parameter instead.  Here, we generate an empty header file to
# satisfy the #include "lcm_export.h" statements within the LCM source code.
genrule(
    name = "lcm_export",
    outs = ["lcm_export.h"],
    cmd = "echo '' > \"$@\"",
    visibility = [])

cc_library(
    name = "lcm",
    srcs = [
        "lcm/eventlog.c",
        "lcm/lcm.c",
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
        "lcm_export.h",  # N.B. This is from lcm_export genrule above.
        "lcm/dbg.h",
        "lcm/eventlog.h",
        "lcm/ioutils.h",
        "lcm/lcm-cpp-impl.hpp",
        "lcm/lcm-cpp.hpp",
        "lcm/lcm.h",
        "lcm/lcm_coretypes.h",
        "lcm/lcm_internal.h",
        "lcm/lcm_version.h",
        "lcm/lcmtypes/channel_port_map_update_t.h",
        "lcm/lcmtypes/channel_to_port_t.h",
        "lcm/ringbuffer.h",
        "lcm/udpm_util.h",
    ],
    includes = ["."],
    copts = ["-Wno-all", "-Wno-deprecated-declarations", "-std=gnu11"],
    # In LCM's build system, these definitions are provided by a generated
    # lcm_export.h file.  For Bazel, we just set them directly as defines.
    defines = [
        "LCM_DEPRECATED=",
        "LCM_DEPRECATED_EXPORT=",
        "LCM_DEPRECATED_NO_EXPORT=",
        "LCM_EXPORT=",
        "LCM_NO_EXPORT=",
    ],
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
        "-Wno-all", "-Wno-format-zero-length", "-std=gnu11",
        # TODO(jwnimmer-tri) This hack should be removed when we ugprade
        # to the latest LCM.
        "-include", "unistd.h"],
    deps = [":lcm"],
)
