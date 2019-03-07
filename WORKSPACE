# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# https://bazel.build/ .

workspace(name = "drake")

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

# These are test repositories only needed for local testing of `external_data`,
# and should not be needed for downstream projects.
load("@drake//tools/external_data/test:external_data_workspace_test.bzl", "add_external_data_test_repositories")  # noqa

add_external_data_test_repositories(__workspace_dir__)

# Add some special heuristic logic for using CLion with Drake.
load("//tools/clion:repository.bzl", "drake_clion_environment")

drake_clion_environment()

load("@bazel_skylib//:lib.bzl", "versions")

# This needs to be in WORKSPACE or a repository rule for native.bazel_version
# to actually be defined. The minimum_bazel_version value should match the
# version passed to the find_package(Bazel) call in the root CMakeLists.txt.
versions.check(minimum_bazel_version = "0.19.2")
