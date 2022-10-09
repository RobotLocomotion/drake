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
        commit = "ed7f7d66505a10ccb44f09bb5739a45e0e6ec73e",
        sha256 = "9da4f12b8108319129109923889d3a54a88ff7c0aa036a72c752b79c9a33050e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
