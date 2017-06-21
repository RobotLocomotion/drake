# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)
load("@drake//tools:python_lint.bzl", "python_lint")
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
        exclude = ["**/CMakeLists.txt"],
    ),
    defines = ["EIGEN_MPL2_ONLY"],
    includes = ["."],
)

cmake_config(
    package = "Eigen3",
    script = "@drake//tools:eigen-create-cps.py",
    version_file = "Eigen/src/Core/util/Macros.h",
)

install_cmake_config(package = "Eigen3")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/eigen3",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/eigen3",
    license_docs = glob(["COPYING.*"]),
    targets = [":eigen"],
    deps = [":install_cmake_config"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = glob(["COPYING.*"]),
    mode = "0644",
    package_dir = "eigen",
)

python_lint()
