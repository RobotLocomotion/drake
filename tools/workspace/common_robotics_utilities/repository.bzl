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
        commit = "775e39374ef230ec0a3085b6c30b0c69d7560fe4",
        sha256 = "446a89507105d72e06c59cb8f74d72c91f6e060f79a5317a02e72f6b007ebb30",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
