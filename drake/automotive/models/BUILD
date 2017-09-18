# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:install_data.bzl", "install_data")
load("//tools:install_data.bzl", "install")

package(
    default_visibility = [":__subpackages__"],
)

genrule(
    name = "speed_bump_genrule",
    srcs = ["speed_bump/speed_bump.yaml"],
    outs = ["speed_bump/speed_bump.obj", "speed_bump/speed_bump.mtl"],
    cmd = " ".join([
        "$(location //drake/automotive/maliput/utility:yaml_to_obj)",
        "--yaml_file '$<'",
        "--obj_dir $(@D)/speed_bump",
        "--obj_file speed_bump",
    ]),
    tools = ["//drake/automotive/maliput/utility:yaml_to_obj"],
)

filegroup(
    name = "all_models",
    srcs = glob(["**"]) + [
        "speed_bump/speed_bump.mtl",
        "speed_bump/speed_bump.obj",
    ],
    visibility = ["//visibility:public"],
)

install_data()

install(
    name = "install_speed_bump",
    data = [
        "speed_bump/speed_bump.mtl",
        "speed_bump/speed_bump.obj",
    ],
    visibility = ["//visibility:public"],
)
