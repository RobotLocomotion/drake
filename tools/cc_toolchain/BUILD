# -*- python -*-

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

# Note that we would typically name this file BUILD.bazel, however Bazel fails
# to find the CROSSTOOL file if we do that.

# The default toolchain selector for Drake.
#
# On macOS we do not support any custom values for the --compiler flag. It
# must be unspecified.  The CROSSTOOL file maps this to the system default
# Clang via osx_cc_wrapper.sh.  Confusingly, Clang is installed on Mac as
# /usr/bin/gcc as well as /usr/bin/clang, and the wrapper script invokes the
# former.
#
# On Linux, we support --compiler strings "clang-4.0" and "gcc-5", each of
# which does exactly what you would expect. "clang-4.0" is the default. No
# wrapper script is needed for any of these compilers, so the cc_toolchain rule
# does not provide one. Support for "gcc-6" is experimental without any
# guarantees. Note that it is not in our supported configurations yet. See
# http://drake.mit.edu/developers.html#supported-configurations for more
# information.
cc_toolchain_suite(
    name = "default-toolchain",
    toolchains = {
        "darwin_x86_64|compiler": "cc_toolchain_apple",
        "k8|clang-4.0": "cc_toolchain_linux",
        "k8|gcc-5": "cc_toolchain_linux",
        "k8|gcc-6": "cc_toolchain_linux",
    },
)

filegroup(
    name = "empty",
    srcs = [],
    visibility = ["//visibility:private"],
)

# This filegroup contains the files on which every toolchain component should
# always depend, regardless of platform. It is a useful place for the outputs
# of rules that check preconditions for the entire build.  (At the moment, we
# do not have any such files, although we once did.)
filegroup(
    name = "universal_toolchain_deps",
    srcs = [
        "//tools/dynamic_analysis:sanitizer_blacklists",
    ],
    visibility = ["//visibility:private"],
)

# A compiler wrapper script that adjusts linker paths on macOS.
filegroup(
    name = "osx_cc_wrapper",
    srcs = [
        "osx_cc_wrapper.sh",
        "//third_party:com_github_bazelbuild_bazel/tools/cpp/osx_cc_wrapper.sh",  # noqa
    ],
    visibility = ["//visibility:private"],
)

filegroup(
    name = "osx_ar_wrapper",
    srcs = ["osx_ar_wrapper.sh"],
    visibility = ["//visibility:private"],
)

filegroup(
    name = "apple_toolchain_deps",
    srcs = [
        ":osx_ar_wrapper",
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
    cpu = "darwin_x86_64",
    dwp_files = ":apple_toolchain_deps",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":apple_toolchain_deps",
    objcopy_files = ":apple_toolchain_deps",
    static_runtime_libs = [":empty"],
    strip_files = ":apple_toolchain_deps",
    supports_param_files = 0,
)

config_setting(
    name = "debug",
    values = {"compilation_mode": "dbg"},
)

config_setting(
    name = "apple_debug",
    values = {
        "compilation_mode": "dbg",
        "cpu": "darwin_x86_64",
    },
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
    name = "clang4.0-linux",
    values = {
        "compiler": "clang-4.0",
        "cpu": "k8",
    },
)

add_lint_tests()
