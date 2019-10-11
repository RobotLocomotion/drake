# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def rules_pkg_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_pkg",  # License: Apache-2.0
        extra_strip_prefix = "pkg",
        commit = "0.2.4",
        sha256 = "08ce92b9aea59ce6d592404de6cdfd7100c1140cdf4d4b9266942c20ec998b27",  # noqa
        mirrors = mirrors,
    )
