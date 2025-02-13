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
        commit = "1414bfe21d7ff7e3f4fec55d82457b6b87186ac2",
        sha256 = "d3b5db55cb5cf10e023e34b874a4a993444a6f31684fabcedfa158b8a38d3c46",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
