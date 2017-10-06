# -*- python -*-

load("@drake//tools:drake.bzl", "drake_generate_file")
load("@drake//tools:generate_export_header.bzl", "generate_export_header")
load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
    "install_files",
)
load("@drake//tools/lint:python_lint.bzl", "python_lint")

package(default_visibility = ["//visibility:public"])

# Generate header to provide ABI export symbols for LCM.
generate_export_header(
    out = "lcm/lcm_export.h",
    lib = "lcm",
    static_define = "LCM_STATIC",
)

# These options are used when building all LCM C/C++ code.
# They do not flow downstream to LCM-using libraries.
LCM_COPTS = [
    "-Wno-all",
    "-Wno-deprecated-declarations",
    "-Wno-format-zero-length",
    "-std=gnu11",
    "-fvisibility=hidden",
]

LCM_PUBLIC_HEADERS = [
    # Public (installed) headers
    "lcm/eventlog.h",
    "lcm/lcm.h",
    "lcm/lcm-cpp.hpp",
    "lcm/lcm-cpp-impl.hpp",
    "lcm/lcm_coretypes.h",
    "lcm/lcm_export.h",  # N.B. This is from generate_export_header above.
    "lcm/lcm_version.h",
]

cc_library(
    name = "lcm",
    srcs = [
        # Sources
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
        # Private headers
        "lcm/dbg.h",
        "lcm/ioutils.h",
        "lcm/lcm_internal.h",
        "lcm/lcmtypes/channel_port_map_update_t.h",
        "lcm/lcmtypes/channel_to_port_t.h",
        "lcm/ringbuffer.h",
        "lcm/udpm_util.h",
    ],
    hdrs = LCM_PUBLIC_HEADERS,
    copts = LCM_COPTS,
    # TODO(jwnimmer-tri): The 'lcm' is needed so we can generate lcm_export.h
    # with the correct path and still include it like '#include "lcm_export.h"'
    # from other LCM headers. However, this "pollutes" the include paths of
    # everyone using LCM. Can we do better?
    includes = [
        ".",
        "lcm",
    ],
    linkstatic = 0,
    deps = ["@glib"],
)

cc_binary(
    name = "lcm-logger",
    srcs = [
        "lcm-logger/glib_util.c",
        "lcm-logger/glib_util.h",
        "lcm-logger/lcm_logger.c",
    ],
    copts = LCM_COPTS,
    deps = [":lcm"],
)

cc_binary(
    name = "lcm-logplayer",
    srcs = [
        "lcm-logger/lcm_logplayer.c",
    ],
    copts = LCM_COPTS,
    deps = [":lcm"],
)

cc_binary(
    name = "lcm-gen",
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
    copts = LCM_COPTS,
    includes = ["."],
    deps = [":lcm"],
)

cc_binary(
    name = "_lcm.so",
    srcs = [
        "lcm-python/module.c",
        "lcm-python/pyeventlog.c",
        "lcm-python/pylcm.c",
        "lcm-python/pylcm.h",
        "lcm-python/pylcm_subscription.c",
        "lcm-python/pylcm_subscription.h",
    ],
    copts = LCM_COPTS,
    linkshared = 1,
    linkstatic = 1,
    visibility = ["//visibility:private"],
    deps = [
        ":lcm",
        "@python",
    ],
)

# Downstream users of lcm-python expect to say "import lcm".  However, in the
# sandbox the python package is located at lcm/lcm-python/lcm/__init__.py to
# match the source tree structure of LCM; without any special help the import
# would fail.
#
# Normally we'd add `imports = ["lcm-python"]` to establish a PYTHONPATH at the
# correct subdirectory, and that almost works.  However, because the external
# is named "lcm", Bazel's auto-generated empty "lcm/__init__.py" at the root of
# the sandbox is found first, and prevents the lcm-python subdirectory from
# ever being found.
#
# To repair this, we provide our own init file at the root of the sandbox that
# overrides the Bazel empty default.  Its implementation just delegates to the
# lcm-python init file.
drake_generate_file(
    name = "init_genrule",
    out = "__init__.py",
    content = "execfile(__path__[0] + \"/lcm-python/lcm/__init__.py\")",
    visibility = ["//visibility:private"],
)

py_library(
    name = "lcm-python-upstream",
    srcs = ["lcm-python/lcm/__init__.py"],  # Actual code from upstream.
    data = [":_lcm.so"],
    visibility = ["//visibility:private"],
)

py_library(
    name = "lcm-python",
    srcs = ["__init__.py"],  # Shim, from the genrule above.
    deps = [":lcm-python-upstream"],
)

java_library(
    name = "lcm-java",
    srcs = glob(["lcm-java/lcm/**/*.java"]),
    javacopts = [
        # Suppressed until lcm-proj/lcm#159 is fixed.
        "-XepDisableAllChecks",
    ],
    runtime_deps = [
        "@com_jidesoft_jide_oss//jar",
        "@commons_io_commons_io//jar",
        "@org_apache_xmlgraphics_xmlgraphics_commons//jar",
    ],
    deps = ["@net_sf_jchart2d_jchart2d//jar"],
)

java_binary(
    name = "lcm-spy",
    jvm_flags = [
        # These flags are copied from the lcm/lcm-java/lcm-spy.sh.in.
        "-Djava.net.preferIPv4Stack=true",
        "-ea",
    ],
    main_class = "lcm.spy.Spy",
    runtime_deps = [
        ":lcm-java",
    ],
)

cmake_config(
    package = "lcm",
    script = "@drake//tools:lcm-create-cps.py",
    version_file = "lcm/lcm_version.h",
)

install_cmake_config(package = "lcm")  # Creates rule :install_cmake_config.

install_files(
    name = "install_extra_cmake",
    dest = "lib/cmake/lcm",
    files = ["lcm-cmake/lcmUtilities.cmake"],
    strip_prefix = ["**/"],
)

install(
    name = "install_python",
    targets = [
        ":_lcm.so",
        ":lcm-python-upstream",
    ],
    library_dest = "lib/python2.7/site-packages/lcm",
    py_strip_prefix = ["lcm-python"],
)

install(
    name = "install",
    targets = [
        ":lcm",
        ":lcm-gen",
        ":lcm-java",
        ":lcm-logger",
        ":lcm-logplayer",
        ":lcm-spy",
    ],
    py_strip_prefix = ["lcm-python"],
    hdrs = LCM_PUBLIC_HEADERS,
    docs = [
        "AUTHORS",
        "COPYING",
        "NEWS",
    ],
    rename = {
        "share/java/liblcm-java.jar": "lcm.jar",
    },
    deps = [
        ":install_cmake_config",
        ":install_extra_cmake",
        ":install_python",
    ],
)

python_lint()
