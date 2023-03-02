# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        # *** BEGIN new_release notice
        # Updating this commit requires local testing; see
        # drake/tools/workspace/meshcat/README.md for details.
        # *** END new_release notice
        commit = "ea474eb1e7b595b45145c8104fc9684d49f15231",
        sha256 = "609988dcb6ca3090121ae0b0a149e0c1fab9656a8f2e867786e666264a1d42ca",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
