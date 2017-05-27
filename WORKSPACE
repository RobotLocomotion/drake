# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

workspace(name = "drake")

# -- START drake externals
# Copy and paste this code section to any external Bazel projects that depend
# on drake and need its dependencies for convenience.
# @note See @drake//tools:externals.bzl for more info.

# Set this to the relative path of drake relative to the active WORKSPACE.
drake_relative_path = "."
# Set this to the absolute path of the CMake build/ directory to consume
# drake-visualizer (#5621). Use __workspace_dir__ if needed.
drake_cmake_install_path = "{}/{}/{}".format(__workspace_dir__,
    drake_relative_path, "build/install")

# # Enable this if drake is being consumed as an external local repository.
# local_repository(
#     name = "drake",
#     path = drake_relative_path,
# )

# Load external rules that are immediately used to define other externals.
load("@drake//tools:externals_rules.bzl", "drake_external_rule_repositories")
drake_external_rule_repositories(
    drake_relative_path = drake_relative_path,
)
# Load external repostories.
load("@drake//tools:externals.bzl", "drake_external_repositories")
drake_external_repositories(
    cmake_install_path = drake_cmake_install_path,
)
# -- END drake externals
