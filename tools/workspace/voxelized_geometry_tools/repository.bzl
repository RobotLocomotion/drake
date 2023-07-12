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
        commit = "732473bb48fb16a748f208773e67902a7324016a",
        sha256 = "fb42fa05896f18517f21fccecd1f742ac26b8ea4e81e15819e3ca9164237d4e1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
