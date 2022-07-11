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
        commit = "9440d72e04ef97c080b59d6d99503ccca58e5419",
        sha256 = "bbeede50eaaee21ac7869a75cb325da1f163418849dae8dba2ed8de4dfd49377",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
