# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

workspace(name = "drake")

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

# These are repositories only needed for local testing (e.g. `external_data`),
# but should not be needed for downstream projects.
load("//tools/workspace:test_repositories.bzl", "add_test_repositories")

add_test_repositories(__workspace_dir__)
