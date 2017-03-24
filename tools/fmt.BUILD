# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "fmt",
    srcs = glob([
        "fmt/*.cc",
        "fmt/*.h",
    ]),
    hdrs = glob(["fmt/*.h"]),
    includes = ["."],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE.rst"],
    package_dir = "fmt",
)
