# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        # Updating this commit requires local testing; see
        # drake/tools/workspace/meshcat/README.md for details.
        commit = "85fd641e8897ffe453502e0ec152225562999b0b",
        sha256 = "6571f294a712216059ce60e22f28db7f37de6b4c45cbb3ccbae6af9a06b63899",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
