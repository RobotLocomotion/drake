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
        commit = "ef83c1215c8286ce7d429d6bea9b082e8c631beb",
        sha256 = "cd72cd9d948cb29bfee85094fca0e4248e8488a9ab10caef23221ca3c6641b4f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
