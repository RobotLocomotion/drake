load("//tools/workspace:github.bzl", "github_archive")

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
        commit = "5c3ae22fa086123eba097569ce6246cfac4b8560",
        sha256 = "4ab8dc6ab4abbc148fad2b6edd943fd72b0687a055211a7ab2ef1094df854f88",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
