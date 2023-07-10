load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def common_robotics_utilities_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "calderpg/common_robotics_utilities",
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake. Tests may have been
        updated in ToyotaResearchInstitute/common_robotics_utilities/test/ or
        ToyotaResearchInstitute/common_robotics_utilities/CMakeLists.txt.ros2
        """,
        commit = "bc04862eabe34cfe0cf70ffedf0c5777b708847e",
        sha256 = "ee96073e7db2f9f04e739163f3f9a84070cf279a5f7fd05424307cfb4bf85795",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
