# -*- python -*-

package(default_visibility = ["//visibility:public"])

sh_binary(
    name = "valgrind",
    srcs = ["valgrind.sh"],
    data = ["valgrind.supp"],
)

sh_binary(
    name = "kcov",
    srcs = ["kcov.sh"],
)

py_binary(
    name = "cmake_configure_file",
    srcs = ["cmake_configure_file.py"],
)

py_binary(
    name = "drakelint",
    srcs = ["drakelint.py"],
)

exports_files(
    ["nlopt-gen-hpp.sh"],
    visibility = ["@nlopt//:__pkg__"],
)

alias(
    name = "buildifier",
    actual = "@com_github_bazelbuild_buildifier//buildifier",
)

# This pulls the otherwise-unused libbot targets into the "//..." label tree.
genrule(
    name = "force_libbot",
    srcs = [
        "@libbot//:bot-lcm-logfilter",
        "@libbot//:bot-lcm-logsplice",
        "@libbot//:bot-lcm-tunnel",
        "@libbot//:bot-spy",
        "@libbot//:lcmtypes_bot2-core-py",
    ],
    outs = ["force_libbot.touch"],
    cmd = "touch $(@)",
    visibility = ["//visibility:private"],
)

# === CROSSTOOL rules ===

# The default toolchain selector for Drake.
#
# On OS X, we do not support any custom values for the --compiler flag. It
# must be unspecified.  The CROSSTOOL file maps this to the system default
# Clang via osx_cc_wrapper.sh.  Confusingly, Clang is installed on Mac as
# /usr/bin/gcc as well as /usr/bin/clang, and the wrapper script invokes the
# former.
#
# On Linux, we support --compiler strings "clang-3.9", "gcc-4.9", and "gcc-5",
# each of which does exactly what you would expect. "clang-3.9" is the default
# for convenience, because the Drake install instructions provide it on both
# Trusty and Xenial. No wrapper script is needed for any of these compilers, so
# the cc_toolchain rule does not provide one. Support for "gcc-6" is
# experimental without any guarantees. Note that it is not in our supported
# configurations yet. See
# http://drake.mit.edu/developers.html#supported-configurations for more
# information.
cc_toolchain_suite(
    name = "default-toolchain",
    toolchains = {
        "darwin|compiler": "cc_toolchain_apple",
        "k8|clang-3.9": "cc_toolchain_linux",
        "k8|gcc-4.9": "cc_toolchain_linux",
        "k8|gcc-5": "cc_toolchain_linux",
        "k8|gcc-6": "cc_toolchain_linux",
    },
)

sh_binary(
    name = "complain_about_ccache",
    srcs = ["complain_about_ccache.sh"],
    visibility = ["//visibility:private"],
)

load("//tools:ccache.bzl", "ccache_is_bad")

# This rule will fail loudly if ccache is on the path.
ccache_is_bad(
    name = "ccache_is_bad",
    visibility = ["//visibility:private"],
)

filegroup(
    name = "empty",
    srcs = [],
    visibility = ["//visibility:private"],
)

# This filegroup contains the files on which every toolchain component should
# always depend, regardless of platform. It is a useful place for the outputs
# of rules that check preconditions for the entire build.
filegroup(
    name = "universal_toolchain_deps",
    srcs = ["ccache.txt"],  # ccache.txt is an output of :ccache_is_bad
    visibility = ["//visibility:private"],
)

# A compiler wrapper script that adjusts linker paths on OS X.
filegroup(
    name = "osx_cc_wrapper",
    srcs = ["third_party/bazel/tools/cpp/osx_cc_wrapper.sh"],
    visibility = ["//visibility:private"],
)

filegroup(
    name = "apple_toolchain_deps",
    srcs = [
        ":osx_cc_wrapper",
        ":universal_toolchain_deps",
    ],
    visibility = ["//visibility:private"],
)

filegroup(
    name = "linux_toolchain_deps",
    srcs = [
        ":universal_toolchain_deps",
    ],
    visibility = ["//visibility:private"],
)

cc_toolchain(
    name = "cc_toolchain_linux",
    all_files = ":linux_toolchain_deps",
    compiler_files = ":linux_toolchain_deps",
    cpu = "k8",
    dwp_files = ":linux_toolchain_deps",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":linux_toolchain_deps",
    objcopy_files = ":linux_toolchain_deps",
    static_runtime_libs = [":empty"],
    strip_files = ":linux_toolchain_deps",
    supports_param_files = 0,
)

cc_toolchain(
    name = "cc_toolchain_apple",
    all_files = ":apple_toolchain_deps",
    compiler_files = ":apple_toolchain_deps",
    cpu = "darwin",
    dwp_files = ":apple_toolchain_deps",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":apple_toolchain_deps",
    objcopy_files = ":apple_toolchain_deps",
    static_runtime_libs = [":empty"],
    strip_files = ":apple_toolchain_deps",
    supports_param_files = 0,
)

# === config_setting rules ===

config_setting(
    name = "debug",
    values = {"compilation_mode": "dbg"},
)

config_setting(
    name = "with_gurobi",
    values = {"define": "WITH_GUROBI=ON"},
)

config_setting(
    name = "with_mosek",
    values = {"define": "WITH_MOSEK=ON"},
)

config_setting(
    name = "with_snopt",
    values = {"define": "WITH_SNOPT=ON"},
)

# IPOPT is an open-source solver, and is included in the Drake build by default.
# The IPOPT build is fragile, so we provide a hidden switch to shut it off for
# developers who don't actually need it.  This is not a supported configuration.
# Use at your own risk: --define=NO_IPOPT=ON
config_setting(
    name = "no_ipopt",
    values = {"define": "NO_IPOPT=ON"},
)

config_setting(
    name = "linux",
    values = {"cpu": "k8"},
)

config_setting(
    name = "apple",
    values = {"cpu": "darwin"},
)

config_setting(
    name = "gcc6-linux",
    values = {
        "compiler": "gcc-6",
        "cpu": "k8",
    },
)

config_setting(
    name = "gcc5-linux",
    values = {
        "compiler": "gcc-5",
        "cpu": "k8",
    },
)

config_setting(
    name = "gcc4.9-linux",
    values = {
        "compiler": "gcc-4.9",
        "cpu": "k8",
    },
)

config_setting(
    name = "clang3.9-linux",
    values = {
        "compiler": "clang-3.9",
        "cpu": "k8",
    },
)

config_setting(
    name = "apple_debug",
    values = {
        "compilation_mode": "dbg",
        "cpu": "darwin",
    },
)
