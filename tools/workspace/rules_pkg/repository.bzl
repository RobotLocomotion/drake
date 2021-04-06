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
        commit = "0.4.0",
        sha256 = "dff10e80f2d58d4ce8434ef794e5f9ec0856f3a355ae41c6056259b65e1ad11a",  # noqa
        mirrors = mirrors,
    )
