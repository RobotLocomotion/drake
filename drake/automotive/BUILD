# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

load("//tools:drake.bzl", "drake_component", "drake_library")
package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/systems:framework"]
drake_library(
    name = "generated_vectors",
    components = [
        "gen/driving_command",
        "gen/euler_floating_joint_state",
        "gen/idm_with_trajectory_agent_state",
        "gen/simple_car_config",
        "gen/simple_car_state",
    ],
    deps = DEPS,
)

DEPS = [":generated_vectors"]
drake_component(name = "curve2", deps = DEPS)
drake_component(name = "idm_with_trajectory_agent", deps = DEPS),
drake_component(name = "simple_car", deps = DEPS)
drake_component(name = "trajectory_car", deps = DEPS + [":curve2"])
