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
        commit = "05862044d453da5807606b8ec295b3ab8f00c493",
        sha256 = "c849371edc83664cddd8d2aad8345e56f430ea9803d84c59936c6e56013675a3",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
