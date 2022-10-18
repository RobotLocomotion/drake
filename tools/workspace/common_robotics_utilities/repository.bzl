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
        repository = "calderpg/common_robotics_utilities",
        # When updating, ensure that any new unit tests are reflected in
        # package.BUILD.bazel and BUILD.bazel in drake. Tests may have been
        # updated in ToyotaResearchInstitute/common_robotics_utilities/test/ or
        # ToyotaResearchInstitute/common_robotics_utilities/CMakeLists.txt.ros2
        commit = "e7eb0bed96fd9e65b0c2bb47c28eb8d163461728",
        sha256 = "2b89992b9b7fe7efb15a52467f51fee461140dfdde9df2acd0e21361d5d79b1c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
