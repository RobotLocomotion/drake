# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that for rules_pkg, we do NOT install its LICENSE file as part of the
# Drake install, because rules_pkg is a build-time tool only.

def rules_pkg_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_pkg",  # License: Apache-2.0
        extra_strip_prefix = "pkg",
        commit = "0.2.5",
        sha256 = "f8bf72e76a15d045f786ef0eba92e073a50bbdbd807d237a43a759d36b1b1e2c",  # noqa
        mirrors = mirrors,
    )
