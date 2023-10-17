load("//tools/workspace:github.bzl", "github_archive")

def voxelized_geometry_tools_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/voxelized_geometry_tools",
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake.
        """,
        commit = "79793a1715642603fb7cba6dbf4c81e9a3bbbfa8",
        sha256 = "c27959721290a6727d3cf12f4822b8112abbb7e16aebc679d7a429f9ed73f017",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
