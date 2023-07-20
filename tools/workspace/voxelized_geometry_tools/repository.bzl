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
        commit = "6fd994565151764d9099819c3b40033179e5e517",
        sha256 = "7f3c7fe0987f4531a671fd244bd43620ccb41d0eae691092a1e589650770c576",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
