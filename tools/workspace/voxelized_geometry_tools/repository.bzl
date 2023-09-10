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
        commit = "0b4f4bbaa3881929045cd9157cd8ce0d9d5911af",
        sha256 = "113014eedde0ad032a5109592ea9b6f6683f7df52d6b150e3c6bbba2aa41b646",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
