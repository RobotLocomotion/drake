# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(default_visibility = ["//visibility:public"])

BOT_CORE_PUBLIC_HDRS = glob(["bot2-core/src/bot_core/*.h"])

cc_library(
    name = "bot2_core",
    srcs = glob(["bot2-core/src/bot_core/*.c"]),
    hdrs = BOT_CORE_PUBLIC_HDRS,
    copts = ["-std=gnu99"],
    includes = ["bot2-core/src"],
    deps = [
        "@bot_core_lcmtypes//:bot_core_lcmtypes_c",
        "@glib",
        "@lcm",
    ],
)

java_library(
    name = "lcmspy_plugins_bot2",
    srcs = glob(["bot2-core/java/src/bot2_spy/*.java"]),
    visibility = ["//visibility:private"],
    deps = [
        "@bot_core_lcmtypes//:bot_core_lcmtypes_java",
        "@lcm//:lcm-java",
    ],
)

java_binary(
    name = "bot-spy",
    main_class = "lcm.spy.Spy",
    runtime_deps = [":lcmspy_plugins_bot2"],
)

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

CMAKE_PACKAGE = "libbot"

cmake_config(package = CMAKE_PACKAGE)

install_cmake_config(
    package = CMAKE_PACKAGE,
    versioned = 0,
)

install(
    name = "install",
    hdrs = BOT_CORE_PUBLIC_HDRS,
    doc_dest = "share/doc/" + CMAKE_PACKAGE,
    hdr_dest = "include/" + CMAKE_PACKAGE,
    hdr_strip_prefix = ["bot2-core/src"],
    license_docs = [
        "LICENSE",
        "@drake//tools:third_party/libbot/LICENSE.ldpc",
    ],
    targets = [
        ":bot-lcm-logfilter",
        ":bot-lcm-logsplice",
        ":bot-lcm-tunnel",
        ":bot-lcm-who",
        ":bot-spy",
        ":bot2_core",
        ":lcmspy_plugins_bot2",
    ],
    deps = [":install_cmake_config"],
)
