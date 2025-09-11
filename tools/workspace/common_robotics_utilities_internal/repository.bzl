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
        commit = "ef45d5b6586531c68e902fea56cf07ec2d4ac859",
        sha256 = "d75ad52d9ff39ae4bb8f502da73d6139a0dba4ee2c4154e5692d087b6ff2e011",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
