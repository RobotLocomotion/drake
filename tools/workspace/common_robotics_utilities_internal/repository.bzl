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
        commit = "53b49a83d26379661993232527030c262de4fc2c",
        sha256 = "46b0bce54b718acd42c7826c5819dd96c84b0af8a38cddc80c9f4ceb0dc9f858",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
