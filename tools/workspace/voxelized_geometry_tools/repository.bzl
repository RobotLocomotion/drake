load("//tools/workspace:github.bzl", "github_archive")

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
        commit = "ae038198528cc1420c3ddac6b5940446636ce1b3",
        sha256 = "6e85c4915f0321e3dd32ba704ba6bd33fca4ee6d0e05c343543802667aec0b37",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
