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
        commit = "a0ca77dda87caa2dd818756c3708ae04f50fcdbd",
        sha256 = "b5a380e6d03a30d3116372cfa3cdfbf3e4e2ed6b5437505c951ac2ffd222a3b3",  # noqa
        build_file = "//tools/workspace/common_robotics_utilities:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
