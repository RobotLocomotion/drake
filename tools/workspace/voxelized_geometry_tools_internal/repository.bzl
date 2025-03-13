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
        commit = "1ccd5f03409f48422c616e1785addadc70ece48c",
        sha256 = "19a69d88dab1ace1298146a87390579bcb1528a784ea404fad231bde72d15701",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
