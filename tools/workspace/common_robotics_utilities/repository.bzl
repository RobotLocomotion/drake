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
        commit = "699a9f4c668c38693a6fd417399fac4263b65d6e",
        sha256 = "7a5c279f46d2a4d48f600755b9bb77bf9a4967d1b1881a207135731af175f3ae",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
