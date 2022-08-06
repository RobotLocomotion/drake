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
        commit = "0f162bb58c89ac0b5392f6d2c49ed7c8601eaa68",
        sha256 = "955b83323e033011e7071a4a1a4ccbba1f2c1b1801dcfe1bbd6bffea3c6f20f8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
