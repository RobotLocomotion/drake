# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

sh_binary(
    name = "valgrind",
    srcs = ["valgrind.sh"],
    data = ["valgrind.supp"],
)

alias(
    name = "buildifier",
    actual = "@com_github_bazelbuild_buildifier//buildifier",
)

cc_toolchain_suite(
    name = "default-toolchain",
    toolchains = {
        "k8|clang-3.9": "cc-compiler-clang-3.9-linux",
    },
)

filegroup(
    name = "empty",
    srcs = [],
)

cc_toolchain(
    name = "cc-compiler-clang-3.9-linux",
    all_files = ":empty",
    compiler_files = ":empty",
    cpu = "k8",
    dwp_files = ":empty",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":empty",
    objcopy_files = ":empty",
    static_runtime_libs = [":empty"],
    strip_files = ":empty",
    supports_param_files = 0,
)

config_setting(
    name = "with_gurobi",
    values = {"define": "WITH_GUROBI=ON"},
)
