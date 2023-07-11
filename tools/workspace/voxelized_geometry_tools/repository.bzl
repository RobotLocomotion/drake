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
        commit = "bf57ef2062e55e85efda05ff56a564ff8f37665f",
        sha256 = "a2b326d6a083338b238e658e047554cd4c95374a11ccb306fccd53e04bc29bec",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
