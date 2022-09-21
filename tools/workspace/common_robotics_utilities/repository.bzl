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
        # package.BUILD.bazel and BUILD.bazel in drake. Tests may have been
        # updated in ToyotaResearchInstitute/common_robotics_utilities/test/ or
        # ToyotaResearchInstitute/common_robotics_utilities/CMakeLists.txt.ros2
        commit = "0c48ca4962055bf861afe7fc3a676a7836ec3b4f",
        sha256 = "5295bbc7a9c460c4bc479d5e15753de2d1034c550f9af320f3350922e43fdd78",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
