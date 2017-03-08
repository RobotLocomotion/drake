# -*- python -*-

load("@//tools:lcm.bzl", "lcm_java_library", "lcm_py_library")

package(default_visibility = ["//visibility:public"])

LCM_SRCS = glob(["bot2-core/lcmtypes/*.lcm"])

LCM_STRUCTS = [
    pathname[len("bot2-core/lcmtypes/bot_core_"):-len(".lcm")]
    for pathname in LCM_SRCS
]

lcm_java_library(
    name = "lcmtypes_bot2-core-java",
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

lcm_py_library(
    name = "lcmtypes_bot2-core-py",
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

java_library(
    name = "lcmspy_plugins_bot2",
    srcs = glob(["bot2-core/java/src/**/*.java"]),
    deps = [
        ":lcmtypes_bot2-core-java",
        "@lcm//:lcm-java",
    ],
)

java_binary(
    name = "bot-spy",
    main_class = "lcm.spy.Spy",
    runtime_deps = [
        ":lcmspy_plugins_bot2",
    ],
)

cc_binary(
    name = "bot-lcm-logfilter",
    srcs = ["bot2-lcm-utils/src/logfilter/lcm-logfilter.c"],
    copts = ["-std=gnu99"],
    deps = ["@lcm//:lcm"],
)

cc_binary(
    name = "bot-lcm-logsplice",
    srcs = ["bot2-lcm-utils/src/logsplice/lcm-logsplice.c"],
    copts = ["-std=gnu99"],
    deps = ["@lcm//:lcm"],
)

cc_library(
    name = "ldpc",
    srcs = [
        "bot2-lcm-utils/src/tunnel/ldpc/getopt.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/getopt.h",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_create_pchk.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_create_pchk.h",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_fec.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_fec.h",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_fec_iterative_decoding.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_matrix_sparse.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_matrix_sparse.h",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_profile.h",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_rand.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_rand.h",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_scheme.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_scheme.h",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_types.h",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_wrapper.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/ldpc_wrapper.h",
        "bot2-lcm-utils/src/tunnel/ldpc/macros.h",
        "bot2-lcm-utils/src/tunnel/ldpc/tools.cpp",
        "bot2-lcm-utils/src/tunnel/ldpc/tools.h",
    ],
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
    deps = ["@lcm//:lcm"],
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
        "@lcm//:lcm",
    ],
)
