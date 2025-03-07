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
        commit = "f5859417e23cc4f0b91760627e5dd639e83fcd60",
        sha256 = "aaff4361b97529eb630aa0c0ea9e2a72aa18989365793c5044342fc3273b1c6a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
