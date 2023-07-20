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
        commit = "d82b404cf097d52ffa67955f2d57687d1851d4fe",
        sha256 = "657a0d53c693f7dab7bd19eb10ecf98fdaf1add94f2f503ea6622c8e3606dd73",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
