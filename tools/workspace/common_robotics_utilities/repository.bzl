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
        commit = "fd9bd40f908e443e3d3b1fe910c8e13975292bd1",
        sha256 = "387beefdd77db9c8455a882f3c6871d7651d93aeb0943544c2be5fba13abf728",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
