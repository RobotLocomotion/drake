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
        commit = "7fb460e6f5f9fd48b785c2826ca22608b899dbf9",
        sha256 = "f4e8c5c9450a9945008ae06e0ae89d946a94e621aef86a0399f2d459cc0824ba",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
