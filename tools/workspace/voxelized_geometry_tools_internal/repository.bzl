load("//tools/workspace:deprecation.bzl", "add_deprecation")
load("//tools/workspace:github.bzl", "github_archive")

def voxelized_geometry_tools_repository(
        name,
        mirrors = None):
    add_deprecation(
        name = name,
        date = "2025-04-01",
        cc_aliases = dict([
            (name, "@voxelized_geometry_tools_internal//:" + name)
            for name in [
                "cl_hpp",
                "cuda_voxelization_helpers",
                "opencl_voxelization_helpers",
                "pointcloud_voxelization",
                "voxelized_geometry_tools",
            ]
        ]),
    )

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
        commit = "81684a253eeee62acac5684410784b0a25b40c3c",
        sha256 = "6ad6dca1381bcf71485b2b18c059217410e8be699ae3b118621577baeb3822bb",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
