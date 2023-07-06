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
        commit = "97101b1399301b2ed978da11d6a9f0cd8ea0856b",
        sha256 = "d86dd9fbdd2f9ea631e3b4e32a8f435bdbeb43de224766bce66ac9c74bbe607c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
