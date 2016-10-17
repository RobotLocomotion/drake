# -*- python -*-
# Copyright 2016 Toyota Research Institute.  All rights reserved.

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

# TODO(jwnimmer-tri) Re-test and get this working.  Approxmate answer:
# genrule(
#     name = "generate_drake_path_cc",
#     srcs = ["common/drake_path.cc.in"],
#     outs = ["common/drake_path.cc"],
#     cmd = "sed 's#@PROJECT_SOURCE_DIR@#???#g' $< > $@",
#     visibility = ["//visibility:public"],
# )

cc_library(
    name = "common",
    hdrs = glob(["common/*.h"]),
    srcs = glob(["common/*.cc"]),
    visibility = ["//visibility:public"],
    deps = ["@eigen//:eigen"],
)

cc_library(
    name = "math",
    hdrs = ["math/autodiff.h"],
    srcs = ["math/autodiff.cc"],
    visibility = ["//visibility:public"],
    deps = ["common"],
)

cc_library(
    name = "systems_framework",
    hdrs = glob(["systems/framework/*.h"]),
    srcs = glob(["systems/framework/*.cc"]),
    visibility = ["//visibility:public"],
    deps = ["common", "math"],
)

cc_library(
    name = "systems_analysis",
    hdrs = glob(["systems/analysis/*.h"]),
    srcs = glob(["systems/analysis/*.cc"]),
    visibility = ["//visibility:public"],
    deps = ["systems_framework"],
)

cc_library(
    name = "automotive",
    hdrs = [
        "automotive/create_trajectory_params.h",
        "automotive/curve2.h",
        "automotive/gen/driving_command.h",
        "automotive/gen/euler_floating_joint_state.h",
        "automotive/gen/idm_with_trajectory_agent_state.h",
        "automotive/gen/simple_car_config.h",
        "automotive/gen/simple_car_state.h",
        "automotive/idm_with_trajectory_agent.h",
        "automotive/idm_with_trajectory_agent-inl.h",
        "automotive/simple_car.h",
        "automotive/simple_car-inl.h",
        "automotive/trajectory_car.h",
    ],
    srcs = [
        "automotive/create_trajectory_params.cc",
        "automotive/curve2.cc",
        "automotive/gen/driving_command.cc",
        "automotive/gen/euler_floating_joint_state.cc",
        "automotive/gen/idm_with_trajectory_agent_state.cc",
        "automotive/gen/simple_car_config.cc",
        "automotive/gen/simple_car_state.cc",
        "automotive/idm_with_trajectory_agent.cc",
        "automotive/simple_car.cc",
        "automotive/trajectory_car.cc",
        ],
    visibility = ["//visibility:public"],
    deps = ["systems_analysis"],
)
