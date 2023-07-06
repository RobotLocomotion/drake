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
        commit = "22e6ad8779c0764fa8b50ff57538bc8abd01d290",
        sha256 = "81c3a6a304470360e8a87a8a4691e48cdaa72dd6eb2710656f3c4b00650b1973",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
