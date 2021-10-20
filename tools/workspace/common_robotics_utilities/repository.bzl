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
        # When updating, ensure that any new unit tests are reflected in
        # package.BUILD.bazel and BUILD.bazel in drake.
        commit = "2d5593f89e4245946b06fdbd5402519f65c15ded",
        sha256 = "8c7a1f79809975cdfeb2e6bf78687f643e446b35983d34447d6cea965486ea4a",  # noqa
        build_file = "@drake//tools/workspace/common_robotics_utilities:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
