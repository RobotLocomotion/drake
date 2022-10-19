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
        commit = "73a4d936fcf72a7319d15f46f3ea493625f29a26",
        sha256 = "4a3cdd7b3d21c0d40477ca87166e6c7e2750118566d4bd83c400f0f1f8d10db1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
