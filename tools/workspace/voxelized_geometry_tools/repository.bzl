load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

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
        commit = "db163d2d2f88a80d482be7fc639eb27f289ac4e4",
        sha256 = "5309e5a65ff3197c22842f339f7ddf0524f1d27517e2ad0f8eccdb394e066e1c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
