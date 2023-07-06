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
        commit = "4ecbcd5c842e5cecbb5dde7cd8d3ca81b8acdb35",
        sha256 = "667d0e75dd194053eaf6db6c1f1f28f908d1ea6d0772d832515d6786a826a1f5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
