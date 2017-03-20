# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "eigen",
    hdrs = glob([
        "Eigen/*",
        "Eigen/**/*.h",
        "unsupported/Eigen/*",
        "unsupported/Eigen/**/*.h",
    ]),
    defines = ["EIGEN_MPL2_ONLY"],
    includes = ["."],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = glob(["COPYING.*"]),
    package_dir = "eigen",
)
