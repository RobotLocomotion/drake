load("//tools/workspace:github.bzl", "github_archive")

def voxelized_geometry_tools_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/voxelized_geometry_tools",
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake.
        """,
        commit = "864b84a0a7d28f4e706bdeae33beee83328c79e0",
        sha256 = "68a0012a36c2190d5ae747abeb7063c66f7d26268777069e2f8a9da28065e4b1",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
