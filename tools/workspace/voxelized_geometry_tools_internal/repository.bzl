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
        commit = "403cc11b4cdb2fd55f204b540e18097a735f0e82",
        sha256 = "7ac7fdd0f9a9170e73cf5a7c3e74694516ddaf890ba7aac24a0e1ba8bcc79daf",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
