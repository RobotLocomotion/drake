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
        commit = "9a4a45da191870215333f830f53d11f564161715",
        sha256 = "c1f0dc130bb1932caf9741391af7b4d4a1b2cecabdd3ab62b7b0eb0eb0769c18",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
