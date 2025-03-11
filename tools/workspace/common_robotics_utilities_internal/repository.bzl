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
        commit = "a508aa0b3c1a76237fbbdb3cc0fa255553377732",
        sha256 = "082be318d48469fe78cffbecbbddd3c300537082b70ffe15d8b695e581fc9c9e",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
