load("//tools/workspace:github.bzl", "github_archive")

def common_robotics_utilities_internal_repository(
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
        commit = "a53f1d831d313cd7ef23835c066461e71b515208",
        sha256 = "7b9e2faa20d602eb1d46dfc4f3ae3d5cf9290723b3dfdffc16338ffa1f0a2131",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
