# -*- python -*-

load("@//tools:install.bzl", "install", "install_files")
load("@//tools:python_lint.bzl", "python_lint")
load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "eigen",
    hdrs = glob(
        include = [
            "Eigen/*",
            "Eigen/**/*.h",
            "unsupported/Eigen/*",
            "unsupported/Eigen/**/*.h",
        ],
        exclude=["**/CMakeLists.txt"],
    ),
    defines = ["EIGEN_MPL2_ONLY"],
    includes = ["."],
)

py_binary(
    name = "create-cps",
    srcs = ["@//tools:eigen-create-cps.py"],
    main = "@//tools:eigen-create-cps.py",
    visibility = ["//visibility:private"],
)

genrule(
    name = "cps",
    srcs = ["Eigen/src/Core/util/Macros.h"],
    outs = ["Eigen3.cps"],
    cmd = "$(location :create-cps) \"$<\" > \"$@\"",
    tools = [":create-cps"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "cmake_exports",
    srcs = ["Eigen3.cps"],
    outs = ["Eigen3Config.cmake"],
    cmd = "$(location @pycps//:cps2cmake_executable) \"$<\" > \"$@\"",
    tools = ["@pycps//:cps2cmake_executable"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "cmake_package_version",
    srcs = ["Eigen3.cps"],
    outs = ["Eigen3ConfigVersion.cmake"],
    cmd = "$(location @pycps//:cps2cmake_executable) --version-check \"$<\" > \"$@\"",
    tools = ["@pycps//:cps2cmake_executable"],
    visibility = ["//visibility:private"],
)

install_files(
    name = "install_cmake",
    dest = "lib/cmake/eigen3",
    files = [
        "Eigen3Config.cmake",
        "Eigen3ConfigVersion.cmake",
    ],
)

install(
    name = "install",
    doc_dest = "share/doc/eigen3",
    docs = glob(["COPYING.*"]),
    targets = ["eigen"],
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/eigen3",
    deps = ["install_cmake"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = glob(["COPYING.*"]),
    mode = "0644",
    package_dir = "eigen",
)

python_lint()
