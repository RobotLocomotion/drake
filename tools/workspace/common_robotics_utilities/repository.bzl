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
        commit = "d8b1a861d07e7c526e2a7dd3123d351498b53636",
        sha256 = "8d3357221aeacabc538391b1ab53bd848a8f29ddae75896912c23b7bb9c3d8d8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
