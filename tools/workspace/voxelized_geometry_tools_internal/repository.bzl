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
        commit = "1c0f3c139bcaa7e37d306882f8077cd67afaad83",
        sha256 = "c9c901eb134e545d94f751bb12c0418523ffd16d3d1a7b057286471b78b0e66a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
