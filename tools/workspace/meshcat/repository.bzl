# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        commit = "fc7e50d232a866129c8dd82b4217335158f3dcf6",
        sha256 = "fc1727dc68986bcd111ea9eb586bdd71a84a50763a45cb82613006fe5ab15104",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
