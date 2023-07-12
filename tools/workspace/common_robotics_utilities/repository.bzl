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
        commit = "8c9f06afc50dbd8a8651ce5936aae0b411b1c45d",
        sha256 = "5405c18d29a0d07947482c3d45d2cc4121713d6ed20f4a7cf7d02319fdac0d92",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
