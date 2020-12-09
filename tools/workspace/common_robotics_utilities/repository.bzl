# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def common_robotics_utilities_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/common_robotics_utilities",
        commit = "c26a80f3bdb9f64c3fab91d5f32c7e3143c052d5",
        sha256 = "0bff43cf91e9e80bdc0e5e6495029105689e197667ab468e965535436cf06035",  # noqa
        build_file = "//tools/workspace/common_robotics_utilities:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
