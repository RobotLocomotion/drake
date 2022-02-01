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
        commit = "d4f55193d9d44ba26c672f00931644c97b7cf3a5",
        sha256 = "bb6fe78f416d19823d93156752647d973f87e3a4f66a53a23f85414c8bb99e80",  # noqa
        build_file = "@drake//tools/workspace/common_robotics_utilities:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
