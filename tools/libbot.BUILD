# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(default_visibility = ["//visibility:public"])

BOT_CORE_PUBLIC_HDRS = [
    "bot2-core/src/bot_core/bot_core.h",
    "bot2-core/src/bot_core/camtrans.h",
    "bot2-core/src/bot_core/circular.h",
    "bot2-core/src/bot_core/color_util.h",
    "bot2-core/src/bot_core/ctrans.h",
    "bot2-core/src/bot_core/fasttrig.h",
    "bot2-core/src/bot_core/fileutils.h",
    "bot2-core/src/bot_core/glib_util.h",
    "bot2-core/src/bot_core/gps_linearize.h",
    "bot2-core/src/bot_core/lcm_util.h",
    "bot2-core/src/bot_core/math_util.h",
    "bot2-core/src/bot_core/minheap.h",
    "bot2-core/src/bot_core/ppm.h",
    "bot2-core/src/bot_core/ptr_circular.h",
    "bot2-core/src/bot_core/rand_util.h",
    "bot2-core/src/bot_core/ringbuf.h",
    "bot2-core/src/bot_core/rotations.h",
    "bot2-core/src/bot_core/serial.h",
    "bot2-core/src/bot_core/set.h",
    "bot2-core/src/bot_core/signal_pipe.h",
    "bot2-core/src/bot_core/small_linalg.h",
    "bot2-core/src/bot_core/ssocket.h",
    "bot2-core/src/bot_core/tictoc.h",
    "bot2-core/src/bot_core/timespec.h",
    "bot2-core/src/bot_core/timestamp.h",
    "bot2-core/src/bot_core/trans.h",
]

cc_library(
    name = "bot2-core",
    srcs = [
        "bot2-core/src/bot_core/camtrans.c",
        "bot2-core/src/bot_core/circular.c",
        "bot2-core/src/bot_core/color_util.c",
        "bot2-core/src/bot_core/ctrans.c",
        "bot2-core/src/bot_core/fasttrig.c",
        "bot2-core/src/bot_core/fileutils.c",
        "bot2-core/src/bot_core/glib_util.c",
        "bot2-core/src/bot_core/gps_linearize.c",
        "bot2-core/src/bot_core/lcm_util.c",
        "bot2-core/src/bot_core/minheap.c",
        "bot2-core/src/bot_core/ppm.c",
        "bot2-core/src/bot_core/ptr_circular.c",
        "bot2-core/src/bot_core/rand_util.c",
        "bot2-core/src/bot_core/ringbuf.c",
        "bot2-core/src/bot_core/rotations.c",
        "bot2-core/src/bot_core/serial.c",
        "bot2-core/src/bot_core/set.c",
        "bot2-core/src/bot_core/signal_pipe.c",
        "bot2-core/src/bot_core/small_linalg.c",
        "bot2-core/src/bot_core/ssocket.c",
        "bot2-core/src/bot_core/tictoc.c",
        "bot2-core/src/bot_core/timespec.c",
        "bot2-core/src/bot_core/timestamp.c",
        "bot2-core/src/bot_core/trans.c",
    ],
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
    srcs = [
        "bot2-core/java/src/bot2_spy/ImagePlugin.java",
        "bot2-core/java/src/bot2_spy/PlanarLidarPlugin.java",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "@bot_core_lcmtypes//:bot_core_lcmtypes_java",
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
    srcs = [
        "bot2-lcm-utils/src/who/lcm-who.c",
        "bot2-lcm-utils/src/who/signal_pipe.c",
        "bot2-lcm-utils/src/who/signal_pipe.h",
    ],
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
    doc_dest = "share/doc/libbot",
    hdr_dest = "include/libbot",
    hdr_strip_prefix = ["bot2-core/src"],
    license_docs = ["LICENSE"],
    targets = [
        ":bot-lcm-logfilter",
        ":bot-lcm-logsplice",
        ":bot-lcm-tunnel",
        ":bot-lcm-who",
        ":bot-spy",
        ":bot2-core",
        ":lcmspy_plugins_bot2",
    ],
    deps = [":install_cmake_config"],
)
