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
        commit = "aae030d383528a187582be83d71c4d931a2eb4eb",
        sha256 = "1b5c57820147df2062a6e88d6b982ebbd578fb60b19c4e87275067f4b64031c3",  # noqa
        build_file = "@drake//tools/workspace/common_robotics_utilities:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
