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
        commit = "b466fbc96db4145a61c8eebb9071bb48660d174d",
        sha256 = "5590cd8668a6fde093ceaf87618ac866dccf3d547422b86c0e7eb2ecd0d3706a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
