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
        commit = "f59b15dfef028f5053770339a25f7014adce60a8",
        sha256 = "17031b345e84026427f49fc65c3e98116dc19d66be098e1b2c111d8d2d386e2b",  # noqa
        build_file = "@drake//tools/workspace/common_robotics_utilities:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
