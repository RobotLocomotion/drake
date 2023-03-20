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
        commit = "89435ebf4621f4e92d3b4c2a18aec7fa91e0cb75",
        sha256 = "7577ccf13db9111a90be946996d512d3a9693b356a7e26e4ab9a1ffb90c8ea2b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
