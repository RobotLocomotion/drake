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
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake. Tests may have been
        updated in ToyotaResearchInstitute/common_robotics_utilities/test/ or
        ToyotaResearchInstitute/common_robotics_utilities/CMakeLists.txt.ros2
        """,
        commit = "a3495f86d14326ec40c59eb730ec06f021a69294",
        sha256 = "e89ebea6e811886e00e78ea73ea345e445130cdaf776c2bc19f107ef52c07b07",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
