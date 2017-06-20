# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)
load(
    "@drake//tools:lcm.bzl",
    "lcm_c_aggregate_header",
    "lcm_c_library",
    "lcm_cc_library",
    "lcm_java_library",
    "lcm_py_library",
)

package(default_visibility = ["//visibility:public"])

# Note that this is only a portion of libbot.

BOT2_CORE_PUBLIC_HDRS = glob(["bot2-core/src/bot_core/*.h"])

cc_library(
    name = "bot2_core",
    srcs = glob(["bot2-core/src/bot_core/*.c"]),
    hdrs = BOT2_CORE_PUBLIC_HDRS,
    copts = ["-std=gnu99"],
    includes = ["bot2-core/src"],
    deps = [
        "@glib",
        "@lcm",
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_c",
    ],
)

java_library(
    name = "lcmspy_plugins_bot2",
    srcs = glob(["bot2-core/java/src/bot2_spy/*.java"]),
    visibility = ["//visibility:private"],
    deps = [
        "@lcm//:lcm-java",
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_java",
    ],
)

java_binary(
    name = "bot-spy",
    main_class = "lcm.spy.Spy",
    runtime_deps = [":lcmspy_plugins_bot2"],
)

# bot2-lcm-utils

cc_binary(
    name = "bot-lcm-logfilter",
    srcs = ["bot2-lcm-utils/src/logfilter/lcm-logfilter.c"],
    copts = ["-std=gnu99"],
    deps = [
        "@glib",
        "@lcm",
    ],
)

cc_binary(
    name = "bot-lcm-logsplice",
    srcs = ["bot2-lcm-utils/src/logsplice/lcm-logsplice.c"],
    copts = ["-std=gnu99"],
    deps = [
        "@glib",
        "@lcm",
    ],
)

cc_library(
    name = "ldpc",
    srcs = glob([
        "bot2-lcm-utils/src/tunnel/ldpc/*.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/*.h",
    ]),
    visibility = ["//visibility:private"],
)

# The C99 (not C++) files from lcm-tunnel.
cc_library(
    name = "tunnel_c99",
    srcs = [
        "bot2-lcm-utils/src/tunnel/introspect.c",
        "bot2-lcm-utils/src/tunnel/introspect.h",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_disconnect_msg_t.c",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_disconnect_msg_t.h",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_params_t.c",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_params_t.h",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_sub_msg_t.c",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_sub_msg_t.h",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_udp_msg_t.c",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_udp_msg_t.h",
        "bot2-lcm-utils/src/tunnel/lcm_util.c",
        "bot2-lcm-utils/src/tunnel/lcm_util.h",
        "bot2-lcm-utils/src/tunnel/signal_pipe.c",
        "bot2-lcm-utils/src/tunnel/signal_pipe.h",
        "bot2-lcm-utils/src/tunnel/ssocket.c",
        "bot2-lcm-utils/src/tunnel/ssocket.h",
    ],
    copts = ["-std=gnu99"],
    visibility = ["//visibility:private"],
    deps = [
        "@glib",
        "@gthread",
        "@lcm",
    ],
)

cc_binary(
    name = "bot-lcm-tunnel",
    srcs = [
        "bot2-lcm-utils/src/tunnel/lcm_tunnel.cpp",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel.h",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_server.cpp",
        "bot2-lcm-utils/src/tunnel/lcm_tunnel_server.h",
    ],
    deps = [
        ":ldpc",
        ":tunnel_c99",
        "@lcm",
    ],
)

cc_binary(
    name = "bot-lcm-who",
    srcs = glob([
        "bot2-lcm-utils/src/who/*.c",
        "bot2-lcm-utils/src/who/*.h",
    ]),
    copts = ["-std=gnu99"],
    deps = [
        "@glib",
        "@lcm",
    ],
)

py_library(
    name = "bot_log2mat",
    srcs = glob(["bot2-lcm-utils/python/src/bot_log2mat/*.py"]),
    imports = ["bot2-lcm-utils/python/src"],
    deps = ["@lcm//:lcm-python"],
)

py_binary(
    name = "bot-log2mat",
    srcs = [":bot_log2mat"],
    main = "bot2-lcm-utils/python/src/bot_log2mat/log_to_mat.py",
)

# bot2-param

BOT2_PARAM_LCM_SRCS = glob(["bot2-param/lcmtypes/*.lcm"])

BOT2_PARAM_LCM_STRUCTS = [
    f.replace("bot2-param/lcmtypes/bot_param_", "").replace(".lcm", "")
    for f in BOT2_PARAM_LCM_SRCS
]

lcm_c_aggregate_header(
    name = "lcmtypes_bot2_param_c_aggregate_header",
    out = "bot2-param/lcmtypes/bot2_param.h",
    lcm_package = "bot_param",
    lcm_srcs = BOT2_PARAM_LCM_SRCS,
    lcm_structs = BOT2_PARAM_LCM_STRUCTS,
    visibility = ["//visibility:private"],
)

lcm_c_library(
    name = "lcmtypes_bot2_param_c",
    aggregate_hdr = ":lcmtypes_bot2_param_c_aggregate_header",
    includes = ["bot2-param"],
    lcm_package = "bot_param",
    lcm_srcs = BOT2_PARAM_LCM_SRCS,
    lcm_structs = BOT2_PARAM_LCM_STRUCTS,
)

lcm_cc_library(
    name = "lcmtypes_bot2_param",
    includes = ["bot2-param"],
    lcm_package = "bot_param",
    lcm_srcs = BOT2_PARAM_LCM_SRCS,
    lcm_structs = BOT2_PARAM_LCM_STRUCTS,
)

lcm_java_library(
    name = "lcmtypes_bot2_param_java",
    lcm_package = "bot_param",
    lcm_srcs = BOT2_PARAM_LCM_SRCS,
    lcm_structs = BOT2_PARAM_LCM_STRUCTS,
)

lcm_py_library(
    name = "lcmtypes_bot2_param_py",
    lcm_package = "bot_param",
    lcm_srcs = BOT2_PARAM_LCM_SRCS,
    lcm_structs = BOT2_PARAM_LCM_STRUCTS,
)

BOT2_PARAM_PUBLIC_HDRS = [
    "bot2-param/src/param_client/param_client.h",
    "bot2-param/src/param_client/param_util.h",
]

BOT2_PARAM_INCLUDE_PREFIX = "bot_param"

# BOT2_PARAM_PUBLIC_HDRS need to be in both src and hdrs because include_prefix
# is being used.
cc_library(
    name = "bot2_param_client",
    srcs = BOT2_PARAM_PUBLIC_HDRS + [
        "bot2-param/src/param_client/misc_utils.h",
        "bot2-param/src/param_client/param_internal.c",
        "bot2-param/src/param_client/param_internal.h",
        "bot2-param/src/param_client/param_util.c",
    ],
    hdrs = BOT2_PARAM_PUBLIC_HDRS,
    copts = ["-std=gnu99"],
    include_prefix = BOT2_PARAM_INCLUDE_PREFIX,
    strip_include_prefix = "bot2-param/src/param_client",
    deps = [
        ":bot2_core",
        ":lcmtypes_bot2_param_c",
        "@glib",
        "@gthread",
        "@lcm",
    ],
)

cc_binary(
    name = "bot-param-dump",
    srcs = [
        "bot2-param/src/param_client/param_internal.h",
        "bot2-param/src/param_tester/param_dump.c",
    ],
    deps = [
        ":bot2_param_client",
        "@glib",
        "@lcm",
    ],
)

cc_binary(
    name = "bot-param-server",
    srcs = [
        "bot2-param/src/param_client/misc_utils.h",
        "bot2-param/src/param_client/param_internal.h",
        "bot2-param/src/param_server/lcm_util.c",
        "bot2-param/src/param_server/lcm_util.h",
        "bot2-param/src/param_server/param_server.c",
    ],
    copts = ["-std=gnu99"],
    deps = [
        ":bot2_param_client",
        "@glib",
        "@lcm",
    ],
)

cc_binary(
    name = "bot-param-tool",
    srcs = [
        "bot2-param/src/param_client/misc_utils.h",
        "bot2-param/src/param_client/param_internal.h",
        "bot2-param/src/param_server/param_tool.c",
    ],
    copts = ["-std=gnu99"],
    deps = [
        ":bot2_param_client",
        "@lcm",
    ],
)

# bot2-frames

BOT2_FRAMES_LCM_SRCS = glob(["bot2-frames/lcmtypes/*.lcm"])

BOT2_FRAMES_LCM_STRUCTS = [
    f.replace("bot2-frames/lcmtypes/bot_frames_", "").replace(".lcm", "")
    for f in BOT2_FRAMES_LCM_SRCS
]

lcm_c_aggregate_header(
    name = "lcmtypes_bot2_frames_c_aggregate_header",
    out = "bot2-frames/lcmtypes/bot2_frames.h",
    lcm_package = "bot_frames",
    lcm_srcs = BOT2_FRAMES_LCM_SRCS,
    lcm_structs = BOT2_FRAMES_LCM_STRUCTS,
    visibility = ["//visibility:private"],
)

lcm_c_library(
    name = "lcmtypes_bot2_frames_c",
    aggregate_hdr = ":lcmtypes_bot2_frames_c_aggregate_header",
    includes = ["bot2-frames"],
    lcm_package = "bot_frames",
    lcm_srcs = BOT2_FRAMES_LCM_SRCS,
    lcm_structs = BOT2_FRAMES_LCM_STRUCTS,
)

lcm_cc_library(
    name = "lcmtypes_bot2_frames",
    includes = ["bot2-frames"],
    lcm_package = "bot_frames",
    lcm_srcs = BOT2_FRAMES_LCM_SRCS,
    lcm_structs = BOT2_FRAMES_LCM_STRUCTS,
)

lcm_java_library(
    name = "lcmtypes_bot2_frames_java",
    lcm_package = "bot_frames",
    lcm_srcs = BOT2_FRAMES_LCM_SRCS,
    lcm_structs = BOT2_FRAMES_LCM_STRUCTS,
)

lcm_py_library(
    name = "lcmtypes_bot2_frames_py",
    lcm_package = "bot_frames",
    lcm_srcs = BOT2_FRAMES_LCM_SRCS,
    lcm_structs = BOT2_FRAMES_LCM_STRUCTS,
)

BOT2_FRAMES_PUBLIC_HDRS = ["bot2-frames/src/bot_frames.h"]

BOT2_FRAMES_INCLUDE_PREFIX = "bot_frames"

# BOT2_FRAMES_PUBLIC_HDRS needs to be in both src and hdrs because
# include_prefix is being used.
cc_library(
    name = "bot2_frames",
    srcs = BOT2_FRAMES_PUBLIC_HDRS + ["bot2-frames/src/bot_frames.c"],
    hdrs = BOT2_FRAMES_PUBLIC_HDRS,
    copts = ["-std=gnu99"],
    include_prefix = BOT2_FRAMES_INCLUDE_PREFIX,
    strip_include_prefix = "bot2-frames/src",
    deps = [
        ":bot2_core",
        ":bot2_param_client",
        ":lcmtypes_bot2_frames_c",
        "@glib",
        "@lcm",
    ],
)

# bot2-lcmgl

BOT2_LCMGL_LCM_SRCS = glob(["bot2-lcmgl/lcmtypes/*.lcm"])

BOT2_LCMGL_LCM_STRUCTS = [
    f.replace("bot2-lcmgl/lcmtypes/bot_lcmgl_", "").replace(".lcm", "")
    for f in BOT2_LCMGL_LCM_SRCS
]

lcm_c_aggregate_header(
    name = "lcmtypes_bot2_lcmgl_c_aggregate_header",
    out = "bot2-lcmgl/lcmtypes/bot2_lcmgl.h",
    lcm_package = "bot_lcmgl",
    lcm_srcs = BOT2_LCMGL_LCM_SRCS,
    lcm_structs = BOT2_LCMGL_LCM_STRUCTS,
    visibility = ["//visibility:private"],
)

lcm_c_library(
    name = "lcmtypes_bot2_lcmgl_c",
    aggregate_hdr = ":lcmtypes_bot2_lcmgl_c_aggregate_header",
    includes = ["bot2-lcmgl"],
    lcm_package = "bot_lcmgl",
    lcm_srcs = BOT2_LCMGL_LCM_SRCS,
    lcm_structs = BOT2_LCMGL_LCM_STRUCTS,
)

lcm_cc_library(
    name = "lcmtypes_bot2_lcmgl",
    includes = ["bot2-lcmgl"],
    lcm_package = "bot_lcmgl",
    lcm_srcs = BOT2_LCMGL_LCM_SRCS,
    lcm_structs = BOT2_LCMGL_LCM_STRUCTS,
)

lcm_java_library(
    name = "lcmtypes_bot2_lcmgl_java",
    lcm_package = "bot_lcmgl",
    lcm_srcs = BOT2_LCMGL_LCM_SRCS,
    lcm_structs = BOT2_LCMGL_LCM_STRUCTS,
)

lcm_py_library(
    name = "lcmtypes_bot2_lcmgl_py",
    lcm_package = "bot_lcmgl",
    lcm_srcs = BOT2_LCMGL_LCM_SRCS,
    lcm_structs = BOT2_LCMGL_LCM_STRUCTS,
)

BOT2_LCMGL_PUBLIC_HDRS = [
    "bot2-lcmgl/src/bot_lcmgl_client/lcmgl.h",
    "bot2-lcmgl/src/bot_lcmgl_render/lcmgl_decode.h",
]

cc_library(
    name = "bot2_lcmgl_client",
    srcs = ["bot2-lcmgl/src/bot_lcmgl_client/lcmgl.c"],
    hdrs = ["bot2-lcmgl/src/bot_lcmgl_client/lcmgl.h"],
    copts = ["-std=gnu99"],
    includes = ["bot2-lcmgl/src"],
    deps = [
        ":lcmtypes_bot2_lcmgl_c",
        "@glib",
        "@lcm",
        "@zlib",
    ],
)

cc_library(
    name = "bot2_lcmgl_render",
    srcs = [
        "bot2-lcmgl/src/bot_lcmgl_client/lcmgl.h",
        "bot2-lcmgl/src/bot_lcmgl_render/lcmgl_decode.c",
    ],
    hdrs = ["bot2-lcmgl/src/bot_lcmgl_render/lcmgl_decode.h"],
    copts = ["-std=gnu99"],
    includes = ["bot2-lcmgl/src"],
    deps = [
        ":bot2_lcmgl_client",
        ":lcmtypes_bot2_lcmgl_c",
        "@zlib",
    ],
)

java_library(
    name = "bot2_lcmgl_java",
    srcs = glob(["bot2-lcmgl/java/src/bot2_lcmgl/*.java"]),
    runtime_deps = ["@lcm//:lcm-java"],
)

py_library(
    name = "bot2_lcmgl_py",
    srcs = glob(["bot2-lcmgl/python/src/bot2_lcmgl/*.py"]),
    imports = ["bot2-lcmgl/python/src"],
    deps = [":lcmtypes_bot2_lcmgl_py"],
)

# install

CMAKE_PACKAGE = "libbot"

cmake_config(package = CMAKE_PACKAGE)

install_cmake_config(
    package = CMAKE_PACKAGE,
    versioned = 0,
)

DOC_DEST = "share/doc/" + CMAKE_PACKAGE

LICENSE_DOCS = [
    "LICENSE",
    "@drake//tools:third_party/libbot/LICENSE.ldpc",
]

install(
    name = "install_lcmtypes",
    doc_dest = DOC_DEST,
    guess_hdrs = "PACKAGE",
    hdr_strip_prefix = [
        "bot2-frames",
        "bot2-lcmgl",
        "bot2-param",
    ],
    license_docs = LICENSE_DOCS,
    py_strip_prefix = [
        "bot2-frames/lcmtypes",
        "bot2-lcmgl/lcmtypes",
        "bot2-param/lcmtypes",
    ],
    rename = {
        "share/java/liblcmtypes_bot2_frames_java.jar": "lcmtypes_bot2_frames.jar",  # noqa
        "share/java/liblcmtypes_bot2_lcmgl_java.jar": "lcmtypes_bot2_lcmgl.jar",  # noqa
        "share/java/liblcmtypes_bot2_param_java.jar": "lcmtypes_bot2_param.jar",  # noqa
    },
    targets = [
        ":lcmtypes_bot2_frames_c",
        ":lcmtypes_bot2_frames_java",
        ":lcmtypes_bot2_frames_py",
        ":lcmtypes_bot2_frames",
        ":lcmtypes_bot2_lcmgl_c",
        ":lcmtypes_bot2_lcmgl_java",
        ":lcmtypes_bot2_lcmgl_py",
        ":lcmtypes_bot2_lcmgl",
        ":lcmtypes_bot2_param_c",
        ":lcmtypes_bot2_param_java",
        ":lcmtypes_bot2_param_py",
        ":lcmtypes_bot2_param",
    ],
)

HDR_DEST = "include/" + CMAKE_PACKAGE

install(
    name = "install_bot2_core",
    hdrs = BOT2_CORE_PUBLIC_HDRS,
    doc_dest = DOC_DEST,
    hdr_dest = HDR_DEST,
    hdr_strip_prefix = ["bot2-core/src"],
    license_docs = LICENSE_DOCS,
    rename = {
        "share/java/liblcmspy_plugins_bot2.jar": "lcmspy_plugins_bot2.jar",
    },
    targets = [
        ":bot-spy",
        ":bot2_core",
        ":lcmspy_plugins_bot2",
    ],
)

install(
    name = "install_bot2_lcm_utils",
    doc_dest = DOC_DEST,
    license_docs = LICENSE_DOCS,
    py_strip_prefix = ["bot2-lcm-utils/python/src"],
    targets = [
        ":bot_log2mat",
        ":bot-lcm-logfilter",
        ":bot-lcm-logsplice",
        ":bot-lcm-tunnel",
        ":bot-lcm-who",
    ],
)

install(
    name = "install_bot2_param",
    hdrs = BOT2_PARAM_PUBLIC_HDRS,
    doc_dest = DOC_DEST,
    hdr_dest = HDR_DEST + "/" + BOT2_PARAM_INCLUDE_PREFIX,
    hdr_strip_prefix = ["bot2-param/src/param_client"],
    license_docs = LICENSE_DOCS,
    targets = [
        ":bot-param-dump",
        ":bot-param-server",
        ":bot-param-tool",
        ":bot2_param_client",
    ],
)

install(
    name = "install_bot2_frames",
    hdrs = BOT2_FRAMES_PUBLIC_HDRS,
    doc_dest = DOC_DEST,
    hdr_dest = HDR_DEST + "/" + BOT2_FRAMES_INCLUDE_PREFIX,
    hdr_strip_prefix = ["bot2-frames/src"],
    license_docs = LICENSE_DOCS,
    targets = [":bot2_frames"],
)

install(
    name = "install_bot2_lcmgl",
    hdrs = BOT2_LCMGL_PUBLIC_HDRS,
    doc_dest = DOC_DEST,
    hdr_dest = HDR_DEST,
    hdr_strip_prefix = ["bot2-lcmgl/src"],
    license_docs = LICENSE_DOCS,
    rename = {
        "share/java/libbot2_lcmgl_java.jar": "bot2_lcmgl.jar",
    },
    targets = [
        ":bot2_lcmgl_client",
        ":bot2_lcmgl_java",
        ":bot2_lcmgl_py",
        ":bot2_lcmgl_render",
    ],
)

install(
    name = "install",
    doc_dest = DOC_DEST,
    license_docs = LICENSE_DOCS,
    deps = [
        ":install_bot2_core",
        ":install_bot2_frames",
        ":install_bot2_lcm_utils",
        ":install_bot2_lcmgl",
        ":install_bot2_param",
        ":install_cmake_config",
        ":install_lcmtypes",
    ],
)
