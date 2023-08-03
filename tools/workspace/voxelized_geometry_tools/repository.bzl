load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

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
        commit = "37122a2d1c7da410965ab0d800b1dbc1c3ffd5fa",
        sha256 = "80046ffe33f6dea3f4dd1b107e1d76a9ed76d21fc18cc9f8337ce641e9dc20cf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
