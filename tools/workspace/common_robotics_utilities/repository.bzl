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
        commit = "d0700edefb72c2393f12ed3bbfbefdd01b02ce6f",
        sha256 = "2cb9a292bb47ba20aa5a89a51199eac35485d8246b027bd1ccf4dac5d98c285b",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
