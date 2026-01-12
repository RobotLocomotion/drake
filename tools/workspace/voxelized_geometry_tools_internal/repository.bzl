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
        commit = "d7f3893645ea0f66b4add600a7dbe2e0bdfb57a2",
        sha256 = "f118f08d5d49d5437b5961c32705c9d38751b1fdc72c81456fda81d55af458c0",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
