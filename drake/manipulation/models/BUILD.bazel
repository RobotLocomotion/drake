# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("@drake//tools:install.bzl", "install")

install(
    name = "install_data",
    visibility = ["//visibility:public"],
    deps = [
        "//drake/manipulation/models/iiwa_description:install_data",
        "//drake/manipulation/models/jaco_description:install_data",
        "//drake/manipulation/models/wsg_50_description:install_data",
    ],
)
