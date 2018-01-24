# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

workspace(name = "drake")

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

# These are test repositories only needed for local testing of `external_data`,
# and should not be needed for downstream projects.
load("@drake//tools/external_data/test:external_data_workspace_test.bzl", "add_external_data_test_repositories")  # noqa

add_external_data_test_repositories(__workspace_dir__)
