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
        commit = "17aabe9898494dbf2d61228ad5cef6fb00d06f94",
        sha256 = "014b9d082d6c65a997aacad56d0eb0c937be190daa62796b583384524c4f9c73",  # noqa
        build_file = "@drake//tools/workspace/common_robotics_utilities:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
