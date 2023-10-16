load("//tools/workspace:github.bzl", "github_archive")

def voxelized_geometry_tools_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "calderpg/voxelized_geometry_tools",
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake.
        """,
        commit = "34a90838f6aa1a15709f1dce5ab7f78a4fc134ff",
        sha256 = "c1dbdb4ca005dede212a0dd1547904a095942de8f6d447894cd67112bd016dc8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
