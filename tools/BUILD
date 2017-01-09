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

alias(
    name = "buildifier",
    actual = "@com_github_bazelbuild_buildifier//buildifier",
)

# A script for adding compiler-specific flags to a CROSSTOOL file.
sh_binary(
    name = "amend_crosstool",
    srcs = ["amend_crosstool.sh"],
)

# Files containing compiler-specific flags, one per line.
filegroup(
    name = "compiler_options",
    srcs = [
        "clang_options.txt",
        "gcc_options.txt",
    ],
)

# A custom CROSSTOOL for clang on Ubuntu.
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

# config_settings available to select properties in Drake targets.
config_setting(
    name = "with_gurobi",
    values = {"define": "WITH_GUROBI=ON"},
)
