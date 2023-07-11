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
        commit = "8bfc88479ae65fd1b09c8184e3bcea909cae42e7",
        sha256 = "8a280599a2e97a722b2c04941021e090d95ca18356d6868bdcab6dce7ef51d1a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
