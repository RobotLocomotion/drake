load("//tools/workspace:deprecation.bzl", "add_deprecation")
load("//tools/workspace:github.bzl", "github_archive")

def common_robotics_utilities_repository(
        name,
        mirrors = None):
    add_deprecation(
        name = name,
        date = "2025-04-01",
        cc_aliases = {
            "common_robotics_utilities": "@common_robotics_utilities_internal//:common_robotics_utilities",  # noqa
        },
    )

def common_robotics_utilities_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/common_robotics_utilities",
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake. Tests may have been
        updated in ToyotaResearchInstitute/common_robotics_utilities/test/ or
        ToyotaResearchInstitute/common_robotics_utilities/CMakeLists.txt.ros2
        """,
        commit = "97592d96fbf5f6e1cbf36492dababc7fc79afbda",
        sha256 = "16cd3513b3d74d19b3c929b083e7daecb8984d6cfdd6a555ba8b29f092a6880c",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
